#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import String

widow = serial.Serial('/dev/ttyUSB1', 115200)

def setup():
    line = widow.read_until('\n')
    while(line != "ok\r\n"):
        rospy.loginfo(line)
        line = widow.read_until('\n')
    rospy.sleep(1)
    widow.write(b'ok\n')

def callback(data):
    
    
    if widow.in_waiting:
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