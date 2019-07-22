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

PIN_IRQ = 16
PIN_CS = 8
PIN_RST = 12
EID = "7D:00:22:EA:82:60:3B:00"
PAN = 0xdeca
dw1000 = None


Send = 0
Acked = 0
Timeouts = 0
Start = None
End = None
ReplyTime = None
RoundTime = None

work = 0

timeout = datetime.utcnow()
timeoutold = datetime.utcnow()
timeoutlimit = timedelta(milliseconds=500)


def interruptCB():
    global Send, Acked, Timeouts, Start, End, ReplyTime, RoundTime

    enableRx = False

    # First read sysstatus and copy it
    dw1000.readRegister(dw1000.sysstatus)
    status = copy.deepcopy(dw1000.sysstatus)

    while(status.getBitsOr(C.SYS_STATUS_ALL_TX + C.SYS_STATUS_ALL_RX_TO + C.SYS_STATUS_ALL_RX_GOOD + C.SYS_STATUS_ALL_RX_ERR)):
        logging.debug(dw1000.getStatusRegisterString())
        if status.getBit(C.RXFCG_BIT):
            logging.debug("RXFCG")
            dw1000.clearStatus(C.SYS_STATUS_ALL_RX_GOOD)

            message = dw1000.getMessage()
            header = MAC.MACHeader.decode(message)
            try:
                logging.debug(message[header.dataOffset:-2])
            except:
                pass

            if status.getBit(C.AAT_BIT) and header.frameControl.ackRequest == 0:
                dw1000.clearStatus([C.AAT_BIT])

            # User code
            # >>>>>>>>
            logging.debug(message)
            if header.frameControl.frameType == MAC.FT_ACK:
                End = dw1000.getReceiveTimestamp()
                Acked += 1
            else:
                logging.debug("!!!!!!!!!!!!!!!!!!!!!!")
                ReplyTime = int(MAC.getPayload(message).decode())
                RoundTime = dw1000.wrapTimestamp(End - Start)
                logging.debug(ReplyTime)
                logging.debug(RoundTime)

            enableRx = True
            # <<<<<<<<<

            # Switch Host Side Receive Buffer Pointer
            if dw1000.dblbuffon:
                dw1000.toggleHSRBP()

        if status.getBit(C.TXFRS_BIT):
            logging.debug("TXFRS")
            dw1000.clearStatus(C.SYS_STATUS_ALL_TX)

            #if status.getBit(C.AAT_BIT) and dw1000.sysctrl.getBit(C.WAIT4RESP_BIT):
            #   dw1000.forceTRxOff()
            #    dw1000.rxreset()

            # User code
            # >>>>>>>>
            Start = dw1000.getTransmitTimestamp()
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
            dw1000.sendMessage(b"\x11\x3b", b"\xca\xde", b"", ackReq=True, wait4resp=True)
            # <<<<<<<<

        if status.getBitsOr(C.SYS_STATUS_ALL_RX_ERR):
            logging.debug("RXERR")
            dw1000.clearStatus(C.SYS_STATUS_ALL_RX_ERR)
            dw1000.sysctrl.setBit(C.WAIT4RESP_BIT, False)

            dw1000.forceTRxOff()
            dw1000.rxreset()

            # User code
            # >>>>>>>>
            dw1000.sendMessage(b"\x11\x3b", b"\xca\xde", b"", ackReq=True, wait4resp=True)
            # <<<<<<<<

        timeoutold = datetime.utcnow()

        dw1000.readRegister(dw1000.sysstatus)
        status = copy.deepcopy(dw1000.sysstatus)

    if enableRx:
        dw1000.newReceive()
        dw1000.startReceive()


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
    global Send, Acked, Timeouts, work, timeout
    try:
        setup()
        Start = datetime.utcnow()
        timeoutold = datetime.utcnow()

        while 1:
            timeout = datetime.utcnow()

            dt = timeout - timeoutold
            if dt > timeoutlimit:
                dw1000.sendMessage(b"\x11\x3b", b"\xca\xde", b"", ackReq=True, wait4resp=True)
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
