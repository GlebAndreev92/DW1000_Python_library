from construct import Struct, BitStruct, BitsInteger, Bit, Padding, Embedded, Byte, Probe, If, IfThenElse, Tell

# Frame Control Frame Type
FT_BEACON = 0b000
FT_DATA = 0b001
FT_ACK = 0b010
FT_MAC = 0b011
FT_INV = 0b111 # invalid

# Frame Control Destination Addressing Mode
AD_NOT = 0b00  # No PAN and address
AD_SAD = 0b10  # 2 octet address
AD_EAD = 0b11  # 8 octet address

# Frame Control Frame Version
IEEE802_15_4_2003 = 0b00
IEEE802_15_4 = 0b01

macShortAddrStruct = Struct(
    'shortAddr'/Byte[2]
)

macExtAddrStruct = Struct(
    'extAddr'/Byte[8]
)

macFrameControlStruct = BitStruct(
    Padding(1),
    'panCompression'/Bit,
    'ackRequest'/Bit,
    'framePending'/Bit,
    'secEnable'/Bit,
    'frameType'/BitsInteger(3),
    'srcAddrMode'/BitsInteger(2),
    'frameVersion'/BitsInteger(2),
    'destAddrMode'/BitsInteger(2),
    Padding(2)
)

macHeaderStruct = Struct(
    'frameControl'/macFrameControlStruct,
    'seqNumber'/Byte,
    'destPAN'/Byte[lambda this: 2 if this.frameControl.destAddrMode & 0b10 else 0],
    'destAddr'/Byte[lambda this: 8 if this.frameControl.destAddrMode == 0b11 else (2 if this.frameControl.destAddrMode == 0b10 else 0)],
    'srcPAN'/Byte[lambda this: 2 if this.frameControl.srcAddrMode & 0b10 and this.frameControl.panCompression == 0 else 0],
    'srcAddr'/Byte[lambda this: 8 if this.frameControl.srcAddrMode == 0b11 else (2 if this.frameControl.srcAddrMode == 0b10 else 0)],
    'auxSecHdr'/Byte[lambda this: 14 if this.frameControl.secEnable else 0], # Spec says 0, 5, 6, 10, 14 but not used here
    'dataOffset'/Tell # Get size of header to later extract data
)

# Beacon

macGTSSpecStruct = BitStruct(
    'gtsDescCnt'/BitsInteger(3),
    Padding(4),
    'gtsPermit'/Bit
)

macGTSDirStruct = BitStruct(
    'gtsDirMask'/Bit[7],
    Padding(1)
)

macGTSListElementStruct = BitStruct(
    'devShortAddr'/BitsInteger(16),
    'gtsStartSlot'/BitsInteger(4),
    'gtsLen'/BitsInteger(4)
)

macGTSInfoStruct = Struct(
    'gtsSpec'/macGTSSpecStruct,
    'gtsDir'/macGTSDirStruct[lambda this: 0 if this.gtsSpec.gtsDescCnt == 0 else 1],
    'gtsList'/macGTSListElementStruct[lambda this: this.gtsSpec.gtsDescCnt]
)

macPendAddrSpecStruct = BitStruct(
    'numShortAddrPending'/BitsInteger(3),
    Padding(1),
    'numExtAddrPending'/BitsInteger(3),
    Padding(1)
)

macPendAddrInfoStruct = Struct(
    'pendAddrSpec'/macPendAddrSpecStruct,
    'addrListShort'/macShortAddrStruct[lambda this: this.pendAddrSpec.numShortAddrPending],
    'addrListExt'/macExtAddrStruct[lambda this: this.pendAddrSpec.numExtAddrPending]
)

macSuperFrameStruct = BitStruct(
    'beaconOrder'/BitsInteger(4),
    'superframeOrder'/BitsInteger(4),
    'finalCAPSlot'/BitsInteger(4),
    'ble'/Bit,
    Padding(1),
    'panCoordinator'/Bit,
    'assocPermit'/Bit
)

macPayloadBeaconStruct = Struct(
    'superframe'/macSuperFrameStruct,
    'gts'/macGTSInfoStruct,
    'pendAddr'/macPendAddrInfoStruct,
    'macBeaconPayload'/Byte[0] # NOT USED NOW
)

class FrameControl:
    def __init__(self):
        self.frameType = 0
        self.secEnable = 0
        self.framePending = 0
        self.ackRequest = 0
        self.panCompression = 0
        self.destAddrMode = 0
        self.frameVersion = 0
        self.srcAddrMode = 0

class MACHeader():
    def __init__(self):
        self.frameControl = FrameControl()
        self.seqNumber = 0
        self.destPAN = []
        self.destAddr = []
        self.srcPAN = []
        self.srcAddr = []
        self.auxSecHdr = []
        self.dataOffset = 0

    def encode(self):
        global macHeaderStruct

        machdrdict = vars(self)
        machdrdict['frameControl'] = vars(machdrdict['frameControl'])
        return macHeaderStruct.build(machdrdict)

    @staticmethod
    def decode(rawhdr):
        global macHeaderStruct

        macHeader = MACHeader()

        con = macHeaderStruct.parse(rawhdr)
        for key, val in con.items():
            if key == "frameControl":
                for key2, val2 in val.items():
                    setattr(macHeader.frameControl, key2, val2)
            setattr(macHeader, key, val)

        return macHeader

    def __str__(self):
        ret = "FrameControl.frametype: {:#05b}\n".format(self.frameControl.frameType)
        ret += "FrameControl.secEnable: {:#03b}\n".format(self.frameControl.secEnable)
        ret += "FrameControl.framePending: {:#03b}\n".format(self.frameControl.framePending)
        ret += "FrameControl.ackRequest: {:#03b}\n".format(self.frameControl.ackRequest)
        ret += "FrameControl.panCompression: {:#03b}\n".format(self.frameControl.panCompression)
        ret += "FrameControl.destAddrMode: {:#04b}\n".format(self.frameControl.destAddrMode)
        ret += "FrameControl.frameVersion: {:#04b}\n".format(self.frameControl.frameVersion)
        ret += "FrameControl.srcAddrMode: {:#04b}\n".format(self.frameControl.srcAddrMode)
        ret += "==\n"
        ret += "Sequence Number: {}\n".format(self.seqNumber)
        ret += "destPAN: {}\n".format(bytes(self.destPAN))
        ret += "destAddr: {}\n".format(bytes(self.destAddr))
        ret += "srcPAN: {}\n".format(bytes(self.srcPAN))
        ret += "srcAddr: {}\n".format(bytes(self.srcAddr))
        ret += "auxSecHdr: {}\n".format(bytes(self.auxSecHdr))
        ret += "dataOffset: {}".format(self.dataOffset)
        
        return ret