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
joystick_threshold = 20
f = open("/dev/hidraw2", "rb")
# Open the file in the read-binary mode
while 1:

    data = struct.unpack('64B',f.read(64))

    # leftStickX = struct.unpack('B', data[1])
    # leftStickY = struct.unpack('B', data[2])
    # rightStickX = struct.unpack('B', data[3])
    # rightStickY = struct.unpack('B', data[4])
    leftStickX = data[1]
    leftStickY = data[2]
    rightStickX = data[3]
    rightStickY = data[4]
    triangle = data[5] >> 7
    circle = data[5] >> 6 & 1
    cross = data[5] >> 5 & 1
    square = data[5] >> 4 & 1
    dpad = data[5] & 0xF
    R3 = data[6] >> 7
    L3 = data[6] >> 6 & 1
    R1 = data[6] >> 1 & 1
    L1 = data[6] & 1
    L2 = data[8]
    R2 = data[9]

    # print("leftStickX: %d" % leftStickX)
    # print("leftStickY: %d" % leftStickY)
    # print("rightStickX: %d" % rightStickX)
    # print("rightStickY: %d" % rightStickY)
    # print("triangle: %d" % triangle)
    # print("circle: %d" % circle)
    # print("cross: %d" % cross)
    # print("square: %d" % square)
    # print("dpad: %d" % dpad)
    # print("R3: %d" % R3)
    # print("L3: %d" % L3)
    # print("R1: %d" % R1)
    # print("L1: %d" % L1)
    # print("R2: %d" % R2)
    # print("L2: %d" % L2)
    #Obtain velocities for x, y and z
    vx = 127 - data[2] #Left Stick Y
    vy = 127 - data[1] #Left Stick X
    vz = 127 - data[4] #Right Stick Y
    
    #Check fro threshold
    vx = 0 if abs(vx) < joystick_threshold else vx
    vy = 0 if abs(vy) < joystick_threshold else vy
    vz = 0 if abs(vz) < joystick_threshold else vz

    print("vx: %d" % vx)
    print("vy: %d" % vy)
    print("vz: %d" % vz)
    print
    