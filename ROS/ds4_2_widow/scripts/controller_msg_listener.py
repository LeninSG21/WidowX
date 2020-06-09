#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import String
import os

os.system("ls -l /dev/ttyUSB*")
tty = raw_input("ttyUSB device number: ")
widow = serial.Serial('/dev/ttyUSB' + tty, 115200)
init = True

def setup():
    line = widow.read_until('\n')
    while(line != "ok\r\n"):
        rospy.loginfo(line)
        line = widow.read_until('\n')
    rospy.sleep(1)
    widow.write(b'ok\n')
    rospy.loginfo("Press PS Button to start!")

def callback(data):
    global init

    if init and data.data != "start":
        return
    else:
        if data.data == "start":
            init = False
        elif widow.in_waiting:
            rospy.loginfo(rospy.get_caller_id() + data.data)
            widow.readline()
            for char in data.data.split(","):
                widow.write(chr(int(char)))


def listener():
    rospy.init_node('controller_msg_listener', anonymous = True)
    setup()
    rospy.Subscriber('controller_message', String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()