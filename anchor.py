"""
This python script uses the DW1000 as a sniffer device
"""

from DW1000 import DW1000
import time
import DW1000Constants as C
import MAC
import logging
import copy

PIN_IRQ = 16
PIN_CS = 8
PIN_RST = 12
EID = "7D:00:22:EA:82:60:3B:11"
PAN = 0xdeca
dw1000 = None

replyTime = 0
timeRecv = 0
address = 0

sendTS = False


def interruptCB():
    global replyTime, timeRecv, address, sendTS

    enableRx = False

    # First read sysstatus and copy it
    dw1000.readRegister(dw1000.sysstatus)
    logging.debug(dw1000.getStatusRegisterString())

    status = copy.deepcopy(dw1000.sysstatus)

    while(status.getBitsOr(C.SYS_STATUS_ALL_TX + C.SYS_STATUS_ALL_RX_TO + C.SYS_STATUS_ALL_RX_GOOD + C.SYS_STATUS_ALL_RX_ERR)):

        if status.getBit(C.RXFCG_BIT):
            logging.debug("RXFCG")
            dw1000.clearStatus(C.SYS_STATUS_ALL_RX_GOOD)

            message = dw1000.getMessage()
            header = MAC.MACHeader.decode(message)

            if status.getBit(C.AAT_BIT) and header.frameControl.ackRequest == 0:
                dw1000.clearStatus([C.AAT_BIT])

            # User code
            # >>>>>>>>
            logging.debug(message)
            timeRecv = dw1000.getReceiveTimestamp()
            address = header.srcAddr
            # <<<<<<<<<

            # Switch Host Side Receive Buffer Pointer
            dw1000.toggleHSRBP()

        if status.getBit(C.TXFRS_BIT):
            logging.debug("TXFRS")
            dw1000.clearStatus(C.SYS_STATUS_ALL_TX)

            if status.getBit(C.AAT_BIT) and dw1000.sysctrl.getBit(C.WAIT4RESP_BIT):
                dw1000.forceTRxOff()
                dw1000.rxreset()

            # User code
            # >>>>>>>>
            if status.getBit(C.AAT_BIT):
                timeSend = dw1000.getTransmitTimestamp()
                replyTime = dw1000.wrapTimestamp(timeSend - timeRecv)
                dw1000.sendMessage(address, b"\xca\xde", str(replyTime).encode(), ackReq=False, wait4resp=False)
            # <<<<<<<<
            enableRx = True

        if status.getBitsOr(C.SYS_STATUS_ALL_RX_TO):
            logging.debug("RXRFTO")
            dw1000.clearStatus([C.RXRFTO_BIT])
            dw1000.sysctrl.setBit(C.WAIT4RESP_BIT, False)

            dw1000.forceTRxOff()
            dw1000.rxreset()

            # User code
            # >>>>>>>>
            enableRx = True
            # <<<<<<<<

        if status.getBitsOr(C.SYS_STATUS_ALL_RX_ERR):
            logging.debug("RXERR")
            dw1000.clearStatus(C.SYS_STATUS_ALL_RX_ERR)
            dw1000.sysctrl.setBit(C.WAIT4RESP_BIT, False)

            dw1000.forceTRxOff()
            dw1000.rxreset()

            # User code
            # >>>>>>>>
            enableRx = True
            # <<<<<<<<

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
    logging.info("############### Anchor ##############")	

    dw1000.generalConfiguration(EID, PAN, C.MODE_STANDARD)
    dw1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)
    dw1000.interruptCallback = interruptCB

    # Enable automatic rx reenable, frame filtering (data frame), auto acknowledge
    # Disable smart tx power
    dw1000.syscfg.setBits((C.DIS_STXP_BIT, C.RXAUTR_BIT, C.FFEN_BIT, C.FFAD_BIT, C.AUTOACK_BIT), True)
    dw1000.writeRegister(dw1000.syscfg)

    # Set HSRBP to ICRBP for double buffering
    dw1000.syncHSRBP()

    # Enable receiver buffer overrun detection, data frame receive
    dw1000.sysmask.clear()
    dw1000.sysmask.setBits((C.MRXOVRR_BIT, C.MRXFCG_BIT, C.MTXFRS_BIT, C.MAAT_BIT), True)
    dw1000.writeRegister(dw1000.sysmask)

    dw1000.clearAllStatus()

    logging.info(dw1000.getDeviceInfoString())


def main():
    try:
        setup()
        dw1000.newReceive()
        dw1000.startReceive()
        
        while 1:
            time.sleep(1)

    except KeyboardInterrupt:
        dw1000.stop()


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    main()