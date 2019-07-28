"""@package docstring
Tag part of SS-TWR system.

This module provides a tag class that ranges to some anchors.
The anchors are specified in the config module (config.py).
"""

import logging
from datetime import datetime

import node
import DW1000Constants as C
from trilaterate import Trilaterator
import config
import MAC

def unixTimestamp():
    """ Get a unix timestamp 

    The timestamp is returned as floating point with whole seconds before the comma.

    Return:
        (float): Timestamp
    """
    return datetime.timestamp(datetime.utcnow())

class Tag(node.Node):
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

        self.dw1000.setFrameWaitTimeout(65535)

        # Enable receiver buffer overrun detection, data frame receive, receive frame wait timeout
        self.dw1000.sysmask.clear()
        self.dw1000.sysmask.setBits((C.MRXOVRR_BIT, C.MRXFCG_BIT, C.MRXRFTO_BIT, C.MTXFRS_BIT), True)
        self.dw1000.writeRegister(self.dw1000.sysmask)

        self.dw1000.clearAllStatus()

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
                    self.dw1000.sendMessage(self.anchor_list[self.anchor_idx], b"\xca\xde", b"", ackReq=True, wait4resp=True)
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
        logging.debug(self.rxrfto_count)
        if self.rxrfto_count >= self.rxrfto_limit:
            self.anchor_tries += 1
            self.rxrfto_count = 0
            self.updateAnchors()
            self.dw1000.sendMessage(self.anchor_list[self.anchor_idx], b"\xca\xde", b"", ackReq=True, wait4resp=True)
            logging.debug("RXRFTO Started ranging to {} with try {}".format(self.anchor_list[self.anchor_idx].hex(), self.anchor_tries))
        else:
            self.dw1000.newReceive()
            self.dw1000.startReceive()

    def cb_rxerr_(self):
        """ Custom rxerr callback """
        self.anchor_next = True
        self.updateAnchors()
        self.dw1000.sendMessage(self.anchor_list[self.anchor_idx], b"\xca\xde", b"", ackReq=True, wait4resp=True)
        logging.debug("RXERR Started ranging to {} with try {}".format(self.anchor_list[self.anchor_idx].hex(), self.anchor_tries))

    def cb_reset_(self):
        """ Custom reset callback """
        self.anchor_tries += 1
        self.dw1000.sendMessage(self.anchor_list[self.anchor_idx], b"\xca\xde", b"", ackReq=True, wait4resp=True)
        logging.debug("Timeout Started ranging to {} with try {}".format(self.anchor_list[self.anchor_idx].hex(), self.anchor_tries))

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