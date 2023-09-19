#!/usr/bin/env python3

import serial
import time
import rospy
from std_msgs.msg import Float32MultiArray
import time

def main():
    SER_PORT = "/dev/ttyACM0"
    BAUD_RATE = 9800
    ser = serial.Serial(SER_PORT,BAUD_RATE)
    time.sleep(1)
    
    rate_hz = 1000
    rospy.init_node('FingerContacts_node', anonymous=True)
    t0 = rospy.get_time()
    pub = rospy.Publisher('/FingerContacts', Float32MultiArray, queue_size=1)
    contact_msg = Float32MultiArray()
    
    r = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        data = ser.readline()
        data = data.decode()
        data_list = data.split(", ")
        try:
            F1 = float(data_list[0])*20
            F2 = float(data_list[1])*20
            t = rospy.get_time() - t0
            contact_msg.data = [F1, F2, 0, 0, 0, F1, F2, 0, 0, 0, t]
            pub.publish(contact_msg)
        except:
            pass
        r.sleep
     

if __name__ == '__main__':
    main()

