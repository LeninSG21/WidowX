import serial
import os
import struct 

#Global variables
f = open("/dev/hidraw2", "rb")
Px = 0
Py = 0
Pz = 0
Gamma = 0
Q5 = 0
joystick_threshold = 10

def getBytes2Send():
    #Read data from the DS4 controller as HID
    data = struct.unpack('64B',f.read(64))

    #Obtain the mapping from the bytes received
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

    #Obtain velocities for x, y and z
    vx = 127 - data[2] #Left Stick Y
    vy = 127 - data[1] #Left Stick X
    vz = 127 - data[4] #Right Stick Y
    
    #Check fro threshold
    vx = 0 if abs(vx) < joystick_threshold else vx
    vy = 0 if abs(vy) < joystick_threshold else vy
    vz = 0 if abs(vz) < joystick_threshold else vz


