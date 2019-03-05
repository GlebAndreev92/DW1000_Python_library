import DW1000Constants as C
from Helper import writeValueToBytes

class DW1000Register:
    def __init__(self, address, subaddress, size):
        self.address = address
        self.subaddress = subaddress
        self.size = size
        self.data = [0] * self.size

    def setBit(self, bit, value):
        shift = bit % 8
        byteidx = int(bit / 8)

        if value:
            self.data[byteidx] |= 0x1 << shift
        else:
            self.data[byteidx] &= ~(0x1 << shift)

    def getBit(self, bit):
        shift = bit % 8
        byteidx = int(bit / 8)

        return (self.data[byteidx] >> shift) & 0x1

    def setBits(self, bits, value):
        for bit in bits:
            self.setBit(bit, value)

    def setAll(self, value):
        for i in range(0, self.size):
            self.data[i] = value

    def clear(self):
        self.setAll(0x00)

    def writeValue(self, value, len=None):
        writeValueToBytes(self.data, value, len if len else self.size)
        
    def __getitem__(self, key):
        return self.data[key]

    def __setitem__(self, key, value):
        self.data[key] = value

    def __str__(self):
        return str(self.data)

    def __repr__(self):
        return repr(self.data)