from ctypes import *
import construct

FT_BEACON = 0b000
FT_DATA = 0b001
FT_ACK = 0b010
FT_MAC = 0b011

PAN_NOT = 0b00  # No PAN and address
PAN_SAD = 0b10  # 2 octet address
PAN_EAD = 0b11  # 8 octet address

class FrameControlFlags(Structure):
    _fields_ = [
        ('frametype', c_uint8, 3),
        ('security', c_uint8, 1),
        ('framepending', c_uint8, 1),
        ('ackreq', c_uint8, 1),
        ('pancompression', c_uint8, 1),
        ('_reserved', c_uint8, 3),
        ('destaddrmode', c_uint8, 2),
        ('frameversion', c_uint8, 2),
        ('srcaddrmode', c_uint8, 2)
    ]

class FrameControl(Union):
    _fields_ = [
        ("byte", c_uint8 * 2),
        ("flags", FrameControlFlags)
    ]

class MACHeader():
    def __init__(self):
        self.frameControl = None
        self.seqNumer = None
        self.destPAN = None
        self.destAddr = None
        self.srcPAN = None
        self.srcAddr = None
        self.auxSec = None

    def encode(self):
        pass

    @staticmethod
    def decode():
        pass