"""@package tag
Tag part of SS-TWR system.

This module provides a tag class that ranges to some anchors.
The anchors are specified in the config module (config.py).
"""

import logging
from datetime import datetime
from threading import Thread, Lock
from http.server import HTTPServer, BaseHTTPRequestHandler

import node
import DW1000Constants as C
from trilaterate import Trilaterator
import config
import MAC

page = (""
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<meta charset=\"UTF-8\">"
        "<meta http-equiv=\"refresh\" content=\"1\">"
        "</head>"
        "<body>"
        "<canvas id=\"uwbmap\" width=\"{}\" height=\"{}\"></canvas>"
        "<script>"
        "var c = document.getElementById(\"uwbmap\");"
        "var ctx = c.getContext(\"2d\");"
        "ctx.fillStyle = \"#000000\";"
        "ctx.fillRect({}, {}, 16, 16);"
        "ctx.fillRect({}, {}, 16, 16);"
        "ctx.fillRect({}, {}, 16, 16);"
        "ctx.fillRect({}, {}, 16, 16);"
        "ctx.fillStyle = \"#FF0000\";"
        "ctx.fillRect({}, {}, 16, 16);"
        "</script>"
        "</body>"
        "</html>"
        "")

def unixTimestamp():
    """
    Get a unix timestamp 

    The timestamp is returned as floating point with whole seconds before the comma.

    Returns:
        (float): Timestamp
    """
    return datetime.timestamp(datetime.utcnow())

class Tag(node.Node):
    """
    Tag class.

    Attributes:
        send: Number of send poll frames
        acked: Number of acked poll frames
        time_poll_send_ts: Timestamp of poll sending
        time_poll_recv_ts: Timestamp of poll receiving
        time_resp_send_ts: Timestamp of response sending
        time_resp_recv_ts: Timestamp of response receiving
        rxrfto_limit: Maximum number of receiver timeouts before a new poll frame is send
        rxrfto_count: Current number of receiver timeouts
        anchor_list: List of anchors used for ranging
        anchor_positions: List of anchor positions
        anchor_distances: Stores distances to anchors, cleared after each round
        anchor_idx: Index of current ranging anchor
        anchor_tries_limit: Maximum number of poll messages per anchor in one round
        anchor_tries: Current number of poll message to the current ranging anchor
        anchor_next: Flag signaling change to next anchor
        trilaterator: Trilaterator object for position calculation
        logfile: Logfile path
        http_thread: Thread handle for the web visualization server
        httpd: Web server
        http_position: Position that is published to the client
    """

    def __init__(self):
        super().__init__()

        # Statistics
        self.send = 0 # Number of send poll frames
        self.acked = 0 # Number of received acks for send polls

        self.time_poll_send_ts = None # Timestamp of poll sending
        self.time_poll_recv_ts = None # Timestamp of poll receiving
        self.time_resp_send_ts = None # Timestamp of response sending
        self.time_resp_recv_ts = None # Timestamp of response receiving

        self.rxrfto_limit = config.rxrfto_limit
        self.rxrfto_count = 0 # Current number of receive frame wait timeouts

        self.anchor_list = config.anchor_list # list of anchors to range to
        self.anchor_positions = config.anchor_positions # list of anchor positions
        self.anchor_distances = {} # measured distances
        self.anchor_idx = 0 # current ranging anchor index
        self.anchor_tries_limit = config.tries_limit # maximum number of poll message resends
        self.anchor_tries = 0 # current number of poll message sends
        self.anchor_next = False # Indicate wanted change anchor_idx to next anchor_idx

        self.trilaterator = Trilaterator() # Trilateror for position estimation

        self.logfile = None # logfile handle

        # Callbacks, see interruptCB
        self.cb_rxfcg = self.cb_rxfcg_
        self.cb_txfrs = self.cb_txfrs_
        self.cb_rxrfto = self.cb_rxrfto_
        self.cb_rxerr = self.cb_rxerr_

        self.cb_irq_while = self.updateAnchors
        self.cb_reset = self.cb_reset_

        self.http_thread = None
        self.httpd = None
        self.http_position = [0., 0., 0.]

    def setup(self):
        """ Tag setup 

        Call after creation of a tag. Set sysctrl and sysmask of DW1000.
        """
        super().setup()

        self.logfile = open(config.logfile, "a")

        self.dw1000.syscfg.setBits((C.DIS_STXP_BIT, C.FFEN_BIT, C.FFAA_BIT, C.FFAD_BIT, C.RXWTOE_BIT, C.AAT_BIT, C.RXAUTR_BIT), True)
        self.dw1000.writeRegister(self.dw1000.syscfg)

        # Set HSRBP to ICRBP for double buffering
        self.dw1000.disableDoubleBuffer()

        self.dw1000.setFrameWaitTimeout(40000)

        # Enable receiver buffer overrun detection, data frame receive, receive frame wait timeout
        self.dw1000.sysmask.clear()
        self.dw1000.sysmask.setBits((C.MRXOVRR_BIT, C.MRXFCG_BIT, C.MRXRFTO_BIT, C.MTXFRS_BIT), True)
        self.dw1000.writeRegister(self.dw1000.sysmask)

        self.dw1000.clearAllStatus()

        if config.webui_enable:
            self.http_thread = Thread(target=self.webserveFunc)
            self.http_thread.start()

    def stop(self):
        super().stop()

        # Shutdown server
        if self.http_thread and self.http_thread.is_alive():
            self.httpd.shutdown_request(None)
            self.httpd.shutdown()
            self.httpd.socket.close()
            self.http_thread.join()

    def computeRange(self):
        """ Calculate range using single sided two way ranging method

        Returns:
            float: Range in meter
        """
        round_time = self.dw1000.wrapTimestamp(self.time_resp_recv_ts - self.time_poll_send_ts)
        reply_time = self.dw1000.wrapTimestamp(self.time_resp_send_ts - self.time_poll_recv_ts)
        tmp_range_ = 0.5 * (round_time - reply_time)
        return (tmp_range_ % C.TIME_OVERFLOW) * C.DISTANCE_OF_RADIO

    def updateAnchors(self):
        """ Handle anchor_idx change

        This is the main function for anchor_idx changes.
        After each round (every anchor in anchor_list is ranged), the number of
        valid ranges is checked and if possible (>3) a position is calculated.
        The position is written to the logfile.
        """
        if self.anchor_next or self.anchor_tries >= self.anchor_tries_limit:
            self.anchor_idx = (self.anchor_idx + 1) % len(self.anchor_list)
            # Calculate position of anchor after trying to measure distance to all anchors
            if self.anchor_idx == 0:
                logging.debug("End of round:\nNumber of distances: {}".format(len(self.anchor_distances)))
                # Extract valid positions and distances
                if len(self.anchor_distances) >= 3:
                    valid_positions = []
                    valid_distances = []
                    for k, v in self.anchor_distances.items():
                        valid_positions.append(self.anchor_positions[k])
                        valid_distances.append(v)
                    # Calculate position
                    position = self.trilaterator.trilaterate(valid_positions, valid_distances, valid_positions[0])
                    self.http_position = position
                    logstring = "{} P {:2} {:2} {:2}\n".format(unixTimestamp(), *position)
                    self.logfile.write(logstring)
                self.anchor_distances.clear()
            # Reset state variables
            self.anchor_tries = 0
            self.anchor_next = False
            self.rxrfto_count = 0

    def cb_rxfcg_(self):
        """ Custom rxfcg callback """
        if self.header.frameControl.frameType == MAC.FT_ACK:
            self.time_resp_recv_ts = self.dw1000.getReceiveTimestamp()
            self.acked += 1
        else:
            try:
                self.time_poll_recv_ts, self.time_resp_send_ts = [int(i) for i in MAC.getPayload(self.message).decode().split(" ")]
                logging.debug("time_poll_recv_ts: {}".format(self.time_poll_recv_ts))
                logging.debug("time_resp_send_ts: {}".format(self.time_resp_send_ts))
                range_ = self.computeRange()
                # Discard unrealistic values
                if range_ > 5000:
                    logging.error("Invalid range")
                    self.anchor_tries += 1
                    self.rxrfto_count = 0
                    self.dw1000.sendMessage(self.anchor_list[self.anchor_idx], config.pan.to_bytes(2, byteorder="little"), b"", ackReq=True, wait4resp=True)
                    logging.debug("RXFCG: Started ranging to {} with try {}".format(self.anchor_list[self.anchor_idx].hex(), self.anchor_tries))
                else:
                    logging.debug("Range to {}: {}".format(self.anchor_list[self.anchor_idx].hex(), range_))
                    self.anchor_distances[self.anchor_idx] = range_
                    self.anchor_next = True
                    logstring = "{} R {} {:4} {} {} {} {}\n".format(unixTimestamp(), self.anchor_list[self.anchor_idx].hex(), range_, self.time_poll_send_ts, self.time_poll_recv_ts, self.time_resp_send_ts, self.time_resp_recv_ts)
                    logfile.write(logstring)
            except:
                pass

        self.enableRx = True

    def cb_txfrs_(self):
        """ Custom txfrs callback """
        self.time_poll_send_ts = self.dw1000.getTransmitTimestamp()
        self.send += 1

    def cb_rxrfto_(self):
        """ Custom rxrfto callback """
        self.rxrfto_count += 1
        if self.rxrfto_count >= self.rxrfto_limit:
            self.anchor_tries += 1
            self.rxrfto_count = 0
            self.updateAnchors()
            self.dw1000.sendMessage(self.anchor_list[self.anchor_idx], config.pan.to_bytes(2, byteorder="little"), b"", ackReq=True, wait4resp=True)
            logging.debug("RXRFTO Started ranging to {} with try {}".format(self.anchor_list[self.anchor_idx].hex(), self.anchor_tries))
        else:
            self.dw1000.newReceive()
            self.dw1000.startReceive()

    def cb_rxerr_(self):
        """ Custom rxerr callback """
        self.anchor_next = True
        self.updateAnchors()
        self.dw1000.sendMessage(self.anchor_list[self.anchor_idx], config.pan.to_bytes(2, byteorder="little"), b"", ackReq=True, wait4resp=True)
        logging.debug("RXERR Started ranging to {} with try {}".format(self.anchor_list[self.anchor_idx].hex(), self.anchor_tries))

    def cb_reset_(self):
        """ Custom reset callback """
        self.anchor_tries += 1
        self.dw1000.sendMessage(self.anchor_list[self.anchor_idx], config.pan.to_bytes(2, byteorder="little"), b"", ackReq=True, wait4resp=True)
        logging.debug("Timeout Started ranging to {} with try {}".format(self.anchor_list[self.anchor_idx].hex(), self.anchor_tries))

    class TagHTTPRequestHandler(BaseHTTPRequestHandler):
        outer = None

        def do_GET(self):
            global page

            width = 800
            height = 800

            min_x = float(min([i[0] for i in config.anchor_positions] + [self.outer.http_position[0]])) - .5
            min_y = float(min([i[1] for i in config.anchor_positions] + [self.outer.http_position[1]])) - .5
            max_x = float(max([i[0] for i in config.anchor_positions] + [self.outer.http_position[0]])) + .5
            max_y = float(max([i[1] for i in config.anchor_positions] + [self.outer.http_position[1]])) + .5

            width_m = max_x - min_x
            height_m = max_y - min_y

            anchor1_x = ((config.anchor_positions[0][0] - min_x) / width_m) * width
            anchor1_y = ((config.anchor_positions[0][1] - min_y) / height_m) * height
            anchor2_x = ((config.anchor_positions[1][0] - min_x) / width_m) * width
            anchor2_y = ((config.anchor_positions[1][1] - min_y) / height_m) * height
            anchor3_x = ((config.anchor_positions[2][0] - min_x) / width_m) * width
            anchor3_y = ((config.anchor_positions[2][1] - min_y) / height_m) * height
            anchor4_x = ((config.anchor_positions[3][0] - min_x) / width_m) * width
            anchor4_y = ((config.anchor_positions[3][1] - min_y) / height_m) * height
            tag_x = ((self.outer.http_position[0] - min_x) / width_m) * width
            tag_y = ((self.outer.http_position[1] - min_y) / height_m) * height

            self.send_response(200)
            self.end_headers()
            self.wfile.write(page.format(width, height,
                                         anchor1_x, anchor1_y,
                                         anchor2_x, anchor2_y,
                                         anchor3_x, anchor3_y,
                                         anchor4_x, anchor4_y,
                                         tag_x, tag_y).encode())

    def webserveFunc(self):
        logging.debug("Starting web server")
        req_handler = self.TagHTTPRequestHandler
        req_handler.outer = self
        self.httpd = HTTPServer(('', 8080), req_handler)
        self.httpd.serve_forever()
        logging.debug("Stopped web server")

def main():
    tag = Tag()
    tag.setup()

    start = datetime.utcnow()

    try:
        tag.run()
    except KeyboardInterrupt:
        tag.dw1000.stop()

    end = datetime.utcnow()

    delta = end - start

    logging.info("Timedelta: {}\nSend: {}\nAcked: {}\nTimeouts: {}\n".format(delta, tag.send, tag.acked, tag.timeouts))

if __name__ == "__main__":
    main()
