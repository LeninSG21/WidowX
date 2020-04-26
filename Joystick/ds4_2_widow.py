# -*- coding: utf-8 -*-
import serial
import struct 
import time

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

#Global variables
f = open("/dev/hidraw2", "rb") #HID File where ds4 event is registered
widow = serial.Serial("/dev/ttyUSB0",115200);
Px = 0 #Point in X from base [cm]
Py = 0 #Point in Y from base [cm]
Pz = 0 #Point in Z from base [cm]
Gamma = 0 #Gamma angle from base_y [°]
Q5 = 512 #Position of the fifth motor from 0 to 1023 (0x3F)
msg = bytearray(6)
Kp = 5/127.5 #[cm/(s*bit)]
Kg = 90.0/255 #[°/(s*bit)]
Kq5 = 512.0/255 #[pos/(s*bit)]
t0 = 0
tf = 0
open_close = 0 #0b01 --> open, 0b10 --> close, 0b00 or 0b11 --> void
option = 0
joystick_threshold = 20

#Limits
xy_lim = 43 #cm
z_lim_up = 52 #cm
z_lim_down = -26 #cm

#Factors
gamma_lim = 90 #°
xy_factor = 127.0/43 # bits/cm
z_factor = 170.0/52 # bits/cm
gamma_factor = 127.0/gamma_lim # bits/°

def calcPoint(vx,vy,vz):
    global Px, Py, Pz

    # print("tf: %f" % tf)
    Px = max(-xy_lim, min(xy_lim, Px + vx * Kp * tf))
    Py = max(-xy_lim, min(xy_lim, Py + vy * Kp * tf))
    Pz = max(z_lim_down, min(z_lim_up, Pz + vz * Kp * tf))
    print("Px: %f"%Px)
    print("Py: %f"%Py)
    print("Pz: %f"%Pz)

def calcGamma(vg, sign):
    global Gamma
    if(sign):
        Gamma -= vg * Kg * tf
    else:
        Gamma += vg * Kg * tf
        
    Gamma = max(-gamma_lim, min(gamma_lim, Gamma))
    print("Gamma: %f"%Gamma)

def calcQ5(vq5, sign):
    global Q5
    if(sign):
        Q5 -= vq5 * Kq5 * tf;
    else:
        Q5 += vq5 * Kq5 * tf;
    
    Q5 = max(0, min(1023, Q5))
    


def buildMSG():
    global msg
    msg[0] = 1<<7 | int(round(abs(Px)*xy_factor)) if Px<0 else int(round(Px*xy_factor))
    msg[1] = 1<<7 | int(round(abs(Py)*xy_factor)) if Py<0 else int(round(Py*xy_factor))
    msg[2] = 0xAA + int(round(abs(Pz)*z_factor)) if Pz<0 else int(round(Pz*z_factor))
    msg[3] = 1<<7 | int(round(abs(Gamma)*gamma_factor)) if Gamma<0 else int(round(Gamma*gamma_factor))
    q5_int = int(round(Q5))
    print("Q5: %d"%q5_int)
    msg[4] = open_close << 6 | q5_int>>4
    msg[5] = (q5_int & 0xf) << 4 | option

def getBytes2Send():
    global option, open_close, tf
    #Read data from the DS4 controller as HID
    data = struct.unpack('64B',f.read(64))

    #Obtain the mapping from the bytes received
    triangle = data[5] >> 7
    circle = data[5] >> 6 & 1
    cross = data[5] >> 5 & 1
    # square = data[5] >> 4 & 1 #unused
    dpad = data[5] & 0xF
    R3 = data[6] >> 7
    L3 = data[6] >> 6 & 1
    R1 = data[6] >> 1 & 1
    L1 = data[6] & 1
    L2 = data[8]
    R2 = data[9]

    #Check for higher priority conditions
    if(L3): #moveRest
        option = 0x01
    elif(R3): #moveHome
        option = 0x02
    elif(triangle): #moveCenter
        option = 0x03
    elif(dpad == 4): #relaxServos
        option = 0x04
    elif(dpad == 0): #torqueServos
        option = 0x05
    else:
        option = 0x00
        #Obtain velocities for x, y, z
        vx = 127.5 - data[2] #Left Stick Y
        vy = 127.5 - data[1] #Left Stick X
        vz = 127.5 - data[4] #Right Stick Y
        
        
        #Check for threshold
        vx = 0 if abs(vx) < joystick_threshold else vx
        vy = 0 if abs(vy) < joystick_threshold else vy
        vz = 0 if abs(vz) < joystick_threshold else vz

        #Obtain new point
        tf = time.time() - t0
        calcPoint(vx,vy,vz)

        #Obtain gamma
        calcGamma(R2, R1)

        #Obtain q5 position
        calcQ5(L2, L1)

        #Obtain open_close
        open_close = cross << 1 | circle 

    buildMSG()


def setup():
    global widow, Px, Py, Pz
    #Wait for first ok
    line = widow.read_until('\n')
    while(line != "ok\r\n"):
        # print(struct.unpack(str(len(line))+'c', line))
        print(line)
        line = widow.read_until('\n')
    # print("yay")
    #Send ok
    widow.write(b'ok\n')
    #Receive points
    Px = float(widow.readline())
    Py = float(widow.readline())
    Pz = float(widow.readline())
    print("Px: %f"%Px)
    print("Py: %f"%Py)
    print("Pz: %f"%Pz)
    #Send ok
    widow.write(b'ok');


def main():
    global t0, widow
    setup()
    #Wait for start button
    data = struct.unpack('64B',f.read(64))
    start = data[7] & 1
    while(start==0):
        data = struct.unpack('64B',f.read(64))
        start = data[7] & 1
    
    delay = time.time()
    while 1:
        t0 = time.time()
        getBytes2Send()
        # print
        widow.write(msg)
        # if(time.time()-delay > 0.01):
        #     widow.write(msg)
        #     delay = time.time()
        #     print
            # for i in range(6):
            #     print("%d: %s" %(i,hex(msg[i])))
            # print

main()