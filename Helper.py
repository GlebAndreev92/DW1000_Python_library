"""@package Helper

This module provides some helper functions to be used by DW1000.
"""

def writeValueToBytes(data, val, n):
    """
    This function writes the value specified and convert it into bytes to write in the array

    Args:
        data: The array you want to write the value into.
        val: The value you want to write in the array.
        n: The size of the array. 

    Returns:
        The modified array of bytes.
    """
    for i in range(0, n):
        data[i] = int(((val >> (i * 8)) & 0xFF))
    return data


def convertStringToByte(string):
    """
    This function converts the string address used for the EUI into a byte array.

    Args:
        string : the string you want to convert into an array of bytes

    Returns:
        The array of bytes

    """
    data = [0] * 8
    for i in range(0, 8):
        data[i] = (int(string[i * 3], 16) << 4) + int(string[i * 3 + 1], 16)
    return data
