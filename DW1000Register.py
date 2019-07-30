"""@package DW1000Register

This module provides a class to manage DWM1000 registers on the host device.
"""

import DW1000Constants as C
from Helper import writeValueToBytes

class DW1000Register:
    """
    DWM1000 register management class.

    Args:
        address: Address of the register (see DW1000Constants)
        subaddress: Offset inside the register (see DW1000Constants)
        size: Size in bytes

    Attributes:
        address: Store the address
        subaddress: Store the subaddress
        size: Store the size
        data: Host side buffer to hold the contents of the referenced ic memory
    """
    def __init__(self, address, subaddress, size):
        self.address = address
        self.subaddress = subaddress
        self.size = size
        self.data = bytearray(self.size)

    def setBit(self, bit, value):
        """
        This function sets a single bit inside the buffer.

        Args:
            bit: Index of the bit
            value: 1 or 0
        """
        shift = bit % 8
        byteidx = int(bit / 8)

        if value:
            self.data[byteidx] |= 0x1 << shift
        else:
            self.data[byteidx] &= ~(0x1 << shift)

    def setBits(self, bits, value):
        """
        This function sets multiple bits.

        Sets all specified bits to the same value. Uses setBit() internally.

        Args:
            bits: List of bits to set
            value: 1 or 0
        """
        for bit in bits:
            self.setBit(bit, value)

    def getBit(self, bit):
        """
        This function extracts the value of a bit at a specific index.

        Args:
            bit: Index of the bit

        Returns:
            Value of the bit
        """
        shift = bit % 8
        byteidx = int(bit / 8)

        return (self.data[byteidx] >> shift) & 0x1

    def getBitsOr(self, bits):
        """
        This function extracts an or reduction of the valuews of multiple bits.

        Uses getBit() internally.

        Args:
            bits: List of bits to get

        Returns:
            Value of all bits ored together
        """
        ret = 0

        for bit in bits:
            ret |= self.getBit(bit)

        return ret

    def setAll(self, value):
        """
        This function sets all bytes in the data buffer to value.

        Args:
            value: Value to use for setting
        """
        for i in range(0, self.size):
            self.data[i] = value

    def clear(self):
        """
        This function clears the whole data buffer to 0.
        """
        self.setAll(0x00)

    def writeValue(self, value, len=None):
        """
        This function memcpy some values into the data buffer.

        Args:
            value (bytes): Source buffer
            len: Number of bytes to copy from value
        """
        writeValueToBytes(self.data, value, len if len else self.size)

    def load(self):
        pass

    def store(self):
        pass

    def __getitem__(self, key):
        return self.data[key]

    def __setitem__(self, key, value):
        self.data[key] = value

    def __str__(self):
        return str(self.data)

    def __repr__(self):
        return repr(self.data)
