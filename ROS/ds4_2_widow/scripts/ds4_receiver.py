#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import struct
import os

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

os.system("ls -l /dev/hidraw*")
hidraw = raw_input("hidraw device number: ")
f = open('/dev/hidraw'+hidraw, 'rb')

joystick_threshold = 10

def readDS4(data):
    msg = ""

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

        vgamma = R2 if R2 > 10 else 0  # Vgamma
        vq5 = L2 if L2 > 10 else 0 # Vq5
        lB = R1 << 7 | L1 << 6 | open_close << 4 | option

        msg = "%d,%d,%d,%d,%d,%d" %(vx,vy,vz, vgamma, vq5, lB)

        return msg
    msg = "0,"*5 + str(option)
    return msg


def talker():
    pub = rospy.Publisher('controller_message', String, queue_size=10)
    rospy.init_node('ds4_receiver', anonymous=True)
    rospy.loginfo("Press PS Button to start!")
    
    while not rospy.is_shutdown():
        data = struct.unpack('64B', f.read(64))
        start = data[7] & 1
        if(start):
            rospy.loginfo("Starting the robotic arm!")
            pub.publish("start")
        else:
            msg = readDS4(data)
            rospy.loginfo(msg)
            pub.publish(msg)
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass