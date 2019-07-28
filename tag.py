"""
This python script uses the DW1000 as a sniffer device
"""

from DW1000 import DW1000
import time
import DW1000Constants as C
import MAC
import logging
from datetime import datetime, timedelta
import copy
import faulthandler
from trilaterate import Trilaterator

PIN_IRQ = 16
PIN_CS = 8
PIN_RST = 12
EID = "7D:00:22:EA:82:60:3B:00"
PAN = 0xdeca
dw1000 = None
logfile = open("/home/pi/uwb.log", "a")

Send = 0
Acked = 0
Timeouts = 0
time_poll_send_ts = None
time_poll_recv_ts = None
time_resp_send_ts = None
time_resp_recv_ts = None

work = 0

timeout = datetime.utcnow()
timeoutold = datetime.utcnow()
timeoutlimit = timedelta(milliseconds=500)

rxrftoLimit = 10
rxrftoCount = 0

anchor_list = [b"\x0a\x3b", b"\x0b\x3b", b"\x0c\x3b", b"\x0d\x3b"]
anchor_positions = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]]
anchor_distances = {}
anchor_idx = 0
anchor_tries = 0
anchor_tries_limit = 10
anchor_next = False

trilaterator = Trilaterator()

def unixTimestamp():
    return datetime.timestamp(datetime.utcnow())

def computeRange():
    roundTime = dw1000.wrapTimestamp(time_resp_recv_ts - time_poll_send_ts)
    replyTime = dw1000.wrapTimestamp(time_resp_send_ts - time_poll_recv_ts)
    timeComputeRangeTs = 0.5 * (roundTime - replyTime)
    return (timeComputeRangeTs % C.TIME_OVERFLOW) * C.DISTANCE_OF_RADIO

def updateAnchors():
    global anchor_list, anchor_distances, anchor_idx, anchor_next, anchor_positions, anchor_tries, anchor_tries_limit, rxrftoCount
    if anchor_next or anchor_tries >= anchor_tries_limit:
        anchor_idx = (anchor_idx + 1) % len(anchor_list)
        # Calculate position of anchor after trying to measure distance to all anchors
        if anchor_idx == 0:
            logging.debug("End of round:\nNumber of distances: {}".format(len(anchor_distances)))
            if len(anchor_distances) >= 3:
                valid_positions = []
                valid_distances = []
                for k, v in anchor_distances.items():
                    valid_positions.append(anchor_positions[k])
                    valid_distances.append(v)
                position = trilaterator.trilaterate(valid_positions, valid_distances, valid_positions[0])
                logstring = "{} P {:2} {:2} {:2}\n".format(unixTimestamp(), *position)
                logfile.write(logstring)
            anchor_distances.clear()
        # Reset number of tries
        anchor_tries = 0
        anchor_next = False
        rxrftoCount = 0

def interruptCB():
    #dw1000.disableInterrupt()
    global Send, Acked, Timeouts, time_poll_send_ts, time_poll_recv_ts, time_resp_send_ts, time_resp_recv_ts, timeoutold, rxrftoCount, rxrftoLimit
    global anchor_list, anchor_distances, anchor_idx, anchor_next, anchor_positions, anchor_tries, anchor_tries_limit

    enableRx = False

    # First read sysstatus and copy it
    dw1000.readRegister(dw1000.sysstatus)
    status = copy.deepcopy(dw1000.sysstatus)

    while(status.getBitsOr(C.SYS_STATUS_ALL_TX + C.SYS_STATUS_ALL_RX_TO + C.SYS_STATUS_ALL_RX_GOOD + C.SYS_STATUS_ALL_RX_ERR)):
        updateAnchors()

        if status.getBit(C.RXFCG_BIT):
            logging.debug("RXFCG")
            dw1000.clearStatus(C.SYS_STATUS_ALL_RX_GOOD)

            message = dw1000.getMessage()
            header = MAC.MACHeader.decode(message)

            if status.getBit(C.AAT_BIT) and header.frameControl.ackRequest == 0:
                dw1000.clearStatus([C.AAT_BIT])

            # User code
            # >>>>>>>>
            if header.frameControl.frameType == MAC.FT_ACK:
                time_resp_recv_ts = dw1000.getReceiveTimestamp()
                Acked += 1
            else:
                try:
                    time_poll_recv_ts, time_resp_send_ts = [int(i) for i in MAC.getPayload(message).decode().split(" ")]
                    logging.debug("time_poll_recv_ts: {}".format(time_poll_recv_ts))
                    logging.debug("time_resp_send_ts: {}".format(time_resp_send_ts))
                    range_ = computeRange()
                    if range_ > 5000:
                        logging.error("Invalid range")
                        anchor_tries += 1
                        rxrftoCount = 0
                        dw1000.sendMessage(anchor_list[anchor_idx], b"\xca\xde", b"", ackReq=True, wait4resp=True)
                        logging.debug("RXFCG: Started ranging to {} with try {}".format(anchor_list[anchor_idx].hex(), anchor_tries))
                    else:
                        logging.debug("Range to {}: {}".format(anchor_list[anchor_idx].hex(), range_))
                        anchor_distances[anchor_idx] = range_
                        anchor_next = True
                        logstring = "{} R {} {:4} {} {} {} {}\n".format(unixTimestamp(), anchor_list[anchor_idx].hex(), range_, time_poll_send_ts, time_poll_recv_ts, time_resp_send_ts, time_resp_recv_ts)
                        logfile.write(logstring)
                except:
                    pass

            enableRx = True
            # <<<<<<<<<

            # Switch Host Side Receive Buffer Pointer
            if dw1000.dblbuffon:
                dw1000.toggleHSRBP()

        if status.getBit(C.TXFRS_BIT):
            logging.debug("TXFRS")
            dw1000.clearStatus(C.SYS_STATUS_ALL_TX)

            if status.getBit(C.AAT_BIT) and dw1000.sysctrl.getBit(C.WAIT4RESP_BIT):
                dw1000.forceTRxOff()
                dw1000.rxreset()

            # User code
            # >>>>>>>>
            time_poll_send_ts = dw1000.getTransmitTimestamp()
            Send += 1
            # <<<<<<<<

        if status.getBitsOr(C.SYS_STATUS_ALL_RX_TO):
            logging.debug("RXRFTO")
            dw1000.clearStatus([C.RXRFTO_BIT])
            dw1000.sysctrl.setBit(C.WAIT4RESP_BIT, False)

            dw1000.forceTRxOff()
            dw1000.rxreset()

            # User code
            # >>>>>>>>
            rxrftoCount += 1
            if rxrftoCount == rxrftoLimit:
                anchor_tries += 1
                updateAnchors()
                dw1000.sendMessage(anchor_list[anchor_idx], b"\xca\xde", b"", ackReq=True, wait4resp=True)
                logging.debug("RXRFTO Started ranging to {} with try {}".format(anchor_list[anchor_idx].hex(), anchor_tries))
            else:
                dw1000.newReceive()
                dw1000.startReceive()
            # <<<<<<<<

        if status.getBitsOr(C.SYS_STATUS_ALL_RX_ERR):
            logging.debug("RXERR")
            dw1000.clearStatus(C.SYS_STATUS_ALL_RX_ERR)
            dw1000.sysctrl.setBit(C.WAIT4RESP_BIT, False)

            dw1000.forceTRxOff()
            dw1000.rxreset()

            # User code
            # >>>>>>>>
            anchor_next = True
            updateAnchors()
            dw1000.sendMessage(anchor_list[anchor_idx], b"\xca\xde", b"", ackReq=True, wait4resp=True)
            logging.debug("RXERR Started ranging to {} with try {}".format(anchor_list[anchor_idx].hex(), anchor_tries))
            # <<<<<<<<

        timeoutold = datetime.utcnow()

        dw1000.readRegister(dw1000.sysstatus)
        status = copy.deepcopy(dw1000.sysstatus)

    if enableRx:
        dw1000.newReceive()
        dw1000.startReceive()

    #dw1000.enableInterrupt()


def setup():
    global dw1000
    dw1000 = DW1000(PIN_CS, PIN_RST, PIN_IRQ)
    dw1000.begin()
    logging.info("DW1000 initialized")
    logging.info("############### Tag ##############")	

    dw1000.generalConfiguration(EID, PAN, C.MODE_STANDARD)
    dw1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)
    dw1000.interruptCallback = interruptCB

    # Enable frame filtering (acknowledge frame), receive frame wait timeout
    # Disable smart tx power
    dw1000.syscfg.setBits((C.DIS_STXP_BIT, C.FFEN_BIT, C.FFAA_BIT, C.FFAD_BIT, C.RXWTOE_BIT, C.AAT_BIT, C.RXAUTR_BIT), True)
    dw1000.writeRegister(dw1000.syscfg)

    # Set HSRBP to ICRBP for double buffering
    dw1000.disableDoubleBuffer()

    dw1000.setFrameWaitTimeout(65535)

    # Enable receiver buffer overrun detection, data frame receive, receive frame wait timeout
    dw1000.sysmask.clear()
    dw1000.sysmask.setBits((C.MRXOVRR_BIT, C.MRXFCG_BIT, C.MRXRFTO_BIT, C.MTXFRS_BIT), True)
    dw1000.writeRegister(dw1000.sysmask)

    dw1000.clearAllStatus()

    logging.info(dw1000.getDeviceInfoString())


def main():
    global Send, Acked, Timeouts, work, timeout, anchor_tries
    try:
        setup()
        Start = datetime.utcnow()
        timeoutold = datetime.utcnow()

        while 1:
            interruptCB()

            timeout = datetime.utcnow()

            dt = timeout - timeoutold
            if dt > timeoutlimit:
                anchor_tries += 1
                dw1000.sendMessage(anchor_list[anchor_idx], b"\xca\xde", b"", ackReq=True, wait4resp=True)
                logging.debug("Timeout Started ranging to {} with try {}".format(anchor_list[anchor_idx].hex(), anchor_tries))
                timeoutold = datetime.utcnow()


    except KeyboardInterrupt:
        dw1000.stop()
        End = datetime.utcnow()

        delta = End - Start

        print("Timedelta: {}\nSend: {}\nAcked: {}\nTimeouts: {}\n".format(delta, Send, Acked, Timeouts))


if __name__ == "__main__":
    faulthandler.enable()
    logging.basicConfig(level=logging.DEBUG)
    main()
