"""@package anchor
Anchor part of SS-TWR system.

This module provides an anchor class that receives and answers to tag poll messages.
"""

import logging

import node
import DW1000Constants as C
import config

class Anchor(node.Node):
    """
    Anchor class.

    Attributes:
        time_recv: DWM1000 timestamp of last received frame
        address: Address of last sender

    """
    def __init__(self):
        super().__init__()

        self.time_recv = 0 # Timestamp of receiving poll message
        self.address = 0 # TODO: Use this field to store address of last message sender

        # Callbacks, see interruptCB
        self.cb_rxfcg = self.cb_rxfcg_
        self.cb_txfrs = self.cb_txfrs_
        self.cb_rxrfto = self.cb_rxrfto_
        self.cb_rxerr = self.cb_rxerr_

        self.cb_reset = self.cb_reset_

    def setup(self):
        """ Anchor setup 

        Call after creation of a anchor. Set sysctrl and sysmask of DW1000.
        """
        super().setup()

        self.dw1000.syscfg.setBits((C.DIS_STXP_BIT, C.RXAUTR_BIT, C.FFEN_BIT, C.FFAD_BIT, C.AUTOACK_BIT), True)
        self.dw1000.writeRegister(self.dw1000.syscfg)

        self.dw1000.enableDoubleBuffer()

        self.dw1000.sysmask.clear()
        self.dw1000.sysmask.setBits((C.MRXOVRR_BIT, C.MRXFCG_BIT, C.MTXFRS_BIT, C.MAAT_BIT), True)
        self.dw1000.writeRegister(self.dw1000.sysmask)

        self.dw1000.clearAllStatus()

    def cb_rxfcg_(self):
        """ Custom rxfcg callback """
        self.time_recv = self.dw1000.getReceiveTimestamp()
        self.address = self.header.srcAddr

    def cb_txfrs_(self):
        """ Custom txfrs callback """
        if self.status.getBit(C.AAT_BIT):
            time_send = self.dw1000.getTransmitTimestamp()
            reply_time = self.dw1000.wrapTimestamp(time_send - self.time_recv)
            logging.debug("Sending reply time {}".format(reply_time))
            self.dw1000.sendMessage(self.address, config.pan.to_bytes(2, byteorder='little'), (str(self.time_recv)+ " " + str(time_send)).encode(), ackReq=False, wait4resp=True, delay=0)
            self.enableRx=False

    def cb_rxrfto_(self):
        """ Custom rxrfto callback """
        self.enableRx = True

    def cb_rxerr_(self):
        """ Custom rxerr callback """
        self.enableRx = True

    def cb_reset_(self):
        """ Custom reset callback """
        self.dw1000.rxreset()
        self.dw1000.idle()
        self.dw1000.newReceive()
        self.dw1000.startReceive()

def main():
    anchor = Anchor()
    anchor.setup()

    try:
        anchor.run()
    except KeyboardInterrupt:
        anchor.dw1000.stop()

if __name__ == "__main__":
    main()
