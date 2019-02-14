"""
This python script is used to configure the DW1000 chip as an anchor for ranging functionalities. It must be used in conjunction with the RangingTAG script. 
It requires the following modules: DW1000, DW1000Constants and monotonic.
"""


from DW1000 import DW1000
import DW1000Constants as C
import time

dw1000 = None

lastActivity = 0
expectedMsgId = C.POLL
protocolFailed = False
sentAck = False
receivedAck = False
LEN_DATA = 16
data = [0] * LEN_DATA
timePollAckSentTS = 0
timePollAckReceivedTS = 0
timePollReceivedTS = 0
timeRangeReceivedTS = 0
timePollSentTS = 0
timeRangeSentTS = 0
timeComputedRangeTS = 0
REPLY_DELAY_TIME_US = 7000 


def millis():
    """
    This function returns the value (in milliseconds) of a clock which never goes backwards. It detects the inactivity of the chip and
    is used to avoid having the chip stuck in an undesirable state.
    """    
    return int(round(time.monotonic() * C.MILLISECONDS))


def handleSent():
    """
    This is a callback called from the module's interrupt handler when a transmission was successful. 
    It sets the sentAck variable as True so the loop can continue.
    """            
    print("Send")
    global sentAck
    sentAck = True


def handleReceived():
    """
    This is a callback called from the module's interrupt handler when a reception was successful. 
    It sets the received receivedAck as True so the loop can continue.
    """       
    print("Received")
    global receivedAck
    receivedAck = True


def noteActivity():
    """
    This function records the time of the last activity so we can know if the device is inactive or not.
    """        
    global lastActivity
    lastActivity = millis()


def resetInactive():
    """
    This function restarts the default polling operation when the device is deemed inactive.
    """    
    global expectedMsgId
    print("reset inactive")    
    expectedMsgId = C.POLL
    receiver()
    noteActivity()


def transmitPollAck():
    """
    This function sends the polling acknowledge message which is used to confirm the reception of the polling message. 
    """        
    global data, dw1000
    dw1000.newTransmit()
    data[0] = C.POLL_ACK
    dw1000.setDelay(REPLY_DELAY_TIME_US, C.MICROSECONDS)
    dw1000.setData(data, LEN_DATA)
    dw1000.startTransmit()


def transmitRangeAcknowledge():
    """
    This functions sends the range acknowledge message which tells the tag that the ranging function was successful and another ranging transmission can begin.
    """
    global data, dw1000
    dw1000.newTransmit()
    data[0] = C.RANGE_REPORT
    dw1000.setData(data, LEN_DATA)
    dw1000.startTransmit()


def transmitRangeFailed():
    """
    This functions sends the range failed message which tells the tag that the ranging function has failed and to start another ranging transmission.
    """    
    global data, dw1000
    dw1000.newTransmit()
    data[0] = C.RANGE_FAILED
    dw1000.setData(data, LEN_DATA)
    dw1000.startTransmit()


def receiver():
    """
    This function configures the chip to prepare for a message reception.
    """    
    global data, dw1000
    dw1000.newReceive()
    dw1000.receivePermanently()
    dw1000.startReceive()


def computeRangeAsymmetric():
    """
    This is the function which calculates the timestamp used to determine the range between the devices.
    """
    global timeComputedRangeTS, dw1000
    round1 = dw1000.wrapTimestamp(timePollAckReceivedTS - timePollSentTS)
    reply1 = dw1000.wrapTimestamp(timePollAckSentTS - timePollReceivedTS)
    round2 = dw1000.wrapTimestamp(timeRangeReceivedTS - timePollAckSentTS)
    reply2 = dw1000.wrapTimestamp(timeRangeSentTS - timePollAckReceivedTS)
    timeComputedRangeTS = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2)


def loop():
    global sentAck, receivedAck, timePollAckSentTS, timePollReceivedTS, timePollSentTS, timePollAckReceivedTS, timeRangeReceivedTS, protocolFailed, data, expectedMsgId, timeRangeSentTS, dw1000
    if (sentAck == False and receivedAck == False):
        if ((millis() - lastActivity) > C.RESET_PERIOD):
            resetInactive()
        return

    if sentAck:
        sentAck = False
        msgId = data[0]
        if msgId == C.POLL_ACK:
            timePollAckSentTS = dw1000.getTransmitTimestamp()
            noteActivity()

    if receivedAck:
        receivedAck = False
        data = dw1000.getData(LEN_DATA)
        msgId = data[0]
        if msgId != expectedMsgId:
            protocolFailed = True
        if msgId == C.POLL:
            protocolFailed = False
            timePollReceivedTS = dw1000.getReceiveTimestamp()
            expectedMsgId = C.RANGE
            transmitPollAck()
            noteActivity()
        elif msgId == C.RANGE:
            timeRangeReceivedTS = dw1000.getReceiveTimestamp()
            expectedMsgId = C.POLL
            if protocolFailed == False:
                timePollSentTS = dw1000.getTimeStamp(data, 1)
                timePollAckReceivedTS = dw1000.getTimeStamp(data, 6)
                timeRangeSentTS = dw1000.getTimeStamp(data, 11)
                computeRangeAsymmetric()
                transmitRangeAcknowledge()
                distance = (timeComputedRangeTS % C.TIME_OVERFLOW) * C.DISTANCE_OF_RADIO
                print("Distance: %.2f m" %(distance))

            else:
                transmitRangeFailed()

            noteActivity()



try:    
    PIN_IRQ = 16
    PIN_SS = 8
    PIN_RST = 12
    dw1000 = DW1000(PIN_SS, PIN_RST, PIN_IRQ)
    dw1000.begin()
    dw1000.setup()
    print("DW1000 initialized")
    print("############### ANCHOR ##############")

    dw1000.generalConfiguration("82:17:5B:D5:A9:9A:E2:00", C.MODE_LONGDATA_RANGE_ACCURACY)
    dw1000.registerCallback("handleSent", handleSent)
    dw1000.registerCallback("handleReceived", handleReceived)
    dw1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)

    receiver()
    noteActivity()
    while 1:
        loop()

except KeyboardInterrupt:
    dw1000.close()
