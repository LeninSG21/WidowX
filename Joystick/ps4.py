import serial
import os
import struct 

"""
byte index

0 --> report index
1 --> L X axis
2 --> L Y axis
3 --> R X axis
4 --> R Y axis
5 --> triangle << 7 | circle << 6 | cross << 5 | square << 4 || D-PAD: hat format
                                                                0x08 --> released
                                                                0x07 --> NW
                                                                0x06 --> W
                                                                0x05 --> SW
                                                                0x04 --> S
                                                                0x03 --> SE
                                                                0x02 --> E
                                                                0x01 --> NE
                                                                0x00 --> N
6 --> R3 << 7 | L3 << 6 | Options << 5 | Share << 4 | R2 << 3 | L2 << 2 | R1 << 1 | L1
7 --> Counter << 2 | T-PAD click << 1 | PS-Button
8 --> L2 Trigger
9 --> R2 Trigger
12 --> Battery Level

Check full HID Data Format for DS4 @ https://www.psdevwiki.com/ps4/DS4-USB
"""

f = open("/dev/hidraw2", "rb")
# Open the file in the read-binary mode
while 1:

    data = f.read(64)

    leftStickX = struct.unpack('B', data[1])
    leftStickY = struct.unpack('B', data[2])
    rightStickX = struct.unpack('B', data[3])
    rightStickY = struct.unpack('B', data[4])
    triangle = data[5] & 7


    print 