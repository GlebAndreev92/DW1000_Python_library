import time
import logging
import copy
import faulthandler
from datetime import datetime, timedelta

from DW1000 import DW1000
import DW1000Constants as C
import MAC
import config

class Node:
    def __init__(self):
        self.dw1000 = None

        self.timeout = time.monotonic()
        self.timeout_old = time.monotonic()
        self.timeout_limit = 0.5

        self.cb_rxfcg = lambda: None
        self.cb_txfrs = lambda: None
        self.cb_rxrfto = lambda: None
        self.cb_rxerr = lambda: None

        self.cb_irq_while = lambda: None
        self.cb_reset = lambda: None

        self.enableRx = False

        self.status = None

        self.message = None
        self.header = None

    def setup(self):
        self.dw1000 = DW1000(config.pin_cs, config.pin_rst, config.pin_irq)
        self.dw1000.begin()
        logging.info("DW1000 initialized")

        self.dw1000.generalConfiguration(config.eid, config.pan, C.MODE_STANDARD)
        self.dw1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)
        self.dw1000.interruptCallback = self.interruptCB

        logging.info(self.dw1000.getDeviceInfoString())

    def run(self):
        self.dw1000.newReceive()
        self.dw1000.startReceive()

        while True:
            # Currently not using irq
            self.interruptCB()

            self.timeout = time.monotonic()
            dt = self.timeout - self.timeout_old
            if dt > self.timeout_limit:
                logging.error("Reset inactive")
                self.cb_reset()
                self.timeout_old = self.timeout

    def stop(self):
        self.dw1000.stop()

    def interruptCB(self):
        self.enableRx = False

        self.dw1000.readRegister(self.dw1000.sysstatus)
        self.status = copy.deepcopy(self.dw1000.sysstatus)

        while(self.status.getBitsOr(C.SYS_STATUS_ALL_TX + C.SYS_STATUS_ALL_RX_TO + C.SYS_STATUS_ALL_RX_GOOD + C.SYS_STATUS_ALL_RX_ERR)):

            self.cb_irq_while()

            if self.status.getBit(C.RXFCG_BIT):
                logging.debug("RXFCG")
                self.dw1000.clearStatus(C.SYS_STATUS_ALL_RX_GOOD)

                self.message = self.dw1000.getMessage()
                self.header = MAC.MACHeader.decode(self.message)

                if self.status.getBit(C.AAT_BIT) and self.header.frameControl.ackRequest == 0:
                    self.dw1000.clearStatus([C.AAT_BIT])
                    self.enableRx = True
                
                # User CB
                self.cb_rxfcg()

                self.dw1000.toggleHSRBP()

            if self.status.getBit(C.TXFRS_BIT):
                logging.debug("TXFRS")
                self.dw1000.clearStatus(C.SYS_STATUS_ALL_TX)
                self.enableRx = True

                if self.status.getBit(C.AAT_BIT) and self.dw1000.sysctrl.getBit(C.WAIT4RESP_BIT):
                    self.dw1000.forceTRxOff()
                    self.dw1000.rxreset()

                # User CB
                self.cb_txfrs()

            if self.status.getBitsOr(C.SYS_STATUS_ALL_RX_TO):
                logging.debug("RXRFTO")
                self.dw1000.clearStatus([C.RXRFTO_BIT])
                self.dw1000.sysctrl.setBit(C.WAIT4RESP_BIT, False)

                self.dw1000.forceTRxOff()
                self.dw1000.rxreset()

                # User CB
                self.cb_rxrfto()

            if self.status.getBitsOr(C.SYS_STATUS_ALL_RX_ERR):
                logging.debug("RXERR")
                self.dw1000.clearStatus(C.SYS_STATUS_ALL_RX_ERR)
                self.dw1000.sysctrl.setBit(C.WAIT4RESP_BIT, False)

                self.dw1000.forceTRxOff()
                self.dw1000.rxreset()

                # User CB
                self.cb_rxerr()

            self.dw1000.readRegister(self.dw1000.sysstatus)
            self.status = copy.deepcopy(self.dw1000.sysstatus)

            self.timeout_old = time.monotonic()

        if self.enableRx:
            self.dw1000.newReceive()
            self.dw1000.startReceive()


