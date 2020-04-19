import serial
import os
import time

import struct 

# struct js_event {
# 		__u32 time;     /* event timestamp in milliseconds */
# 		__s16 value;    /* value */
# 		__u8 type;      /* event type */
# 		__u8 number;    /* axis/button number */
# 	};

f = open( "/dev/input/event22", "rb" ); 
# Open the file in the read-binary mode
while 1:

    data = f.read(8)
    # print len(data)
    timestamp = data[:4]
    # print len(timestamp)
    other = data[4:]
    print struct.unpack('I', timestamp)
    print struct.unpack('hBB', other)
    # timestamp =  struct.unpack('I',f.read(4))
    # value = struct.unpack('h', f.read(2))
    # type_ = struct.unpack('B', f.read(1))
    # number = struct.unpack('B',f.read(1))
    # print("Timestamp: " + str(timestamp))
    # print("Value: " + str(value))
    # print("Type: " + str(type_))
    # print("Number: " + str(number))