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

f = open('/dev/hidraw2', 'rb')
# widow = serial.Serial("/dev/ttyUSB0", 115200)

joystick_threshold = 20


def readDS4(data):
    # Define byte array
    msg = bytearray(6)
    # msg[0] = 9 #reserved char
    # msg[7] = '>'
    # Obtain the mapping from the bytes received
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


    # Check for higher priority conditions
    if(L3):  # moveRest
        option = 0x01
    elif(R3):  # moveHome
        option = 0x02
    elif(triangle):  # moveCenter
        option = 0x03
    elif(dpad == 4):  # relaxServos
        option = 0x04
    elif(dpad == 0):  # torqueServos
        option = 0x05
    elif(dpad == 2):
        option = 0x06 #Move by point
    elif(dpad == 6):
        option = 0x07 #Move arm from {1}
    else:
        option = 0x00
        # Obtain velocities for x, y, z
        vx = data[2] if (data[2] >> 7) else (0x7F - data[2])
        vy = data[1] if (data[1] >> 7) else (0x7F - data[1])
        vz = data[4] if (data[4] >> 7) else (0x7F - data[4])

        vx = 0 if (vx & 0x7F) < joystick_threshold else vx
        vy = 0 if (vy & 0x7F) < joystick_threshold else vy
        vz = 0 if (vz & 0x7F) < joystick_threshold else vz

        open_close = cross << 1 | circle

        msg[0] = vx
        msg[1] = vy
        msg[2] = vz
        msg[3] = R2 if R2 > 10 else 0  # Vgamma
        msg[4] = L2 if L2 > 10 else 0 # Vq5
        msg[5] = R1 << 7 | L1 << 6 | open_close << 4 | option

        return msg
    msg[5] = option
    return msg


def setup():

    line = widow.read_until('\n')
    while(line != "ok\r\n"):
        print(line)
        line = widow.read_until('\n')
    print("Press PS Button to start")
    data = struct.unpack('64B', f.read(64))
    start = data[7] & 1
    while(start == 0):
        data = struct.unpack('64B', f.read(64))
        start = data[7] & 1
    widow.write(b'ok\n')


def main():
    # setup()
    print("Starting loop")
    while 1:
        # Read data from the DS4 controller as HID
        data = struct.unpack('64B', f.read(64))
        if widow.in_waiting:
            widow.readline()
            msg = readDS4(data)
            print(struct.unpack('6B', msg))
            widow.write(msg)


main()