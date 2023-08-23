# -*- coding: utf-8 -*-

from iiwaPy3 import iiwaPy3
from math import pi
from math import sin
from math import cos
from math import atan2
from math import sqrt
import time
from datetime import datetime
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
import numpy as np

class KukaControl:
    def __init__(self):          
        
        # Setup connection to iiwa
        self.IP_of_robot = '172.31.1.148'        
        self.connection_state = False
        self.connect_to_iiwa()


        try:
            print('Kuka Arm: Stopping servo')
            self.iiwa.realTime_startDirectServoCartesian()               # Stop direct servo
        except:
            pass

        try:
            print('Kuka Arm: Stopping servo')
            self.iiwa.realTime_stopDirectServoCartesian()               # Stop direct servo
        except:
            pass
        
        try:
            print('Kuka Arm: Disconnecting')
            self.disconnect_from_iiwa()                                 # Disconnect
        except:
            pass
        
    def connect_to_iiwa(self):
        # Check if already connected to the robot
        if self.connection_state:
            print("Kuka Arm: Already connected to the robot on IP: " + self.IP_of_robot)
            return

        # If the program made it to here, then there is no connection yet so try to connect
        print("Kuka Arm: Connecting to robot at ip: " + self.IP_of_robot)
        try:
            self.iiwa = iiwaPy3(self.IP_of_robot)
            self.connection_state = True
            print("Kuka Arm: Connection established successfully")
            return
            
        except:
            print("Kuka Arm: Error, could not connect at the specified IP")
            return            

    def disconnect_from_iiwa(self):
        # Check if there is an active connection
        print("Kuka Arm: Disconnecting from robot")
        if self.connection_state == False:
            print("Kuka Arm: Already offline")
            return

        # If made it to here, then there is an active connection so try to disconnect
        try:
            self.iiwa.close()
            self.connection_state = False
            print("Kuka Arm: Disconnected successfully")
            
        except:
            print("Kuka Arm: Error could not disconnect")
            return





if __name__ == '__main__':
    KukaControl()
    
