#!/usr/bin/env python

import serial
import time
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
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
    pub_egg = rospy.Publisher('/EggCrack', Int16, queue_size=1)
    contact_msg = Float32MultiArray()
    egg_msg = Int16()
    egg_msg.data = 0
    
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
            
            egg = int(data_list[2])
            egg_msg.data = egg
##            if egg > 0:
##                egg_msg.data = 1
            pub_egg.publish(egg_msg)

                
            
        except:
            pass
        r.sleep
     

if __name__ == '__main__':
    main()

