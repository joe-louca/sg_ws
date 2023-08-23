# -*- coding: utf-8 -*-

from iiwaPy3 import iiwaPy3
from math import pi
from math import sin
from math import cos
from math import atan2
from math import asin
from math import sqrt
import time
from datetime import datetime
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import urllib.request

class KukaControl:
    def __init__(self):          
        self.start_time = datetime.now()                            # Start time for getSecs()
        self.frame_id = 0
        self.timestamp = rospy.Time.now()
        
        # Setup connection to iiwa
        self.IP_of_robot = '172.31.1.148'        
        self.connection_state = False
        self.connect_to_iiwa()

        # Set some parameters
        timestep = 0.01#0.002           # Secs (Good at 0.01) - Docs says >20ms for servo control #0.001s, 200Hz bit shakey
        rate_hz = 200#100               # Hz
        kuka_joints_msg = Float32MultiArray() # Initialise msg to publish
        cmd = [None, None, None, None, None, None]
        kuka_position = [None, None, None, None, None, None]

        try:        
            # Move to home pos
            print('Kuka Arm: Moving to home position')
            start_pos = [0, -25*pi/180, 0, 75*pi/180, 0, -75*pi/180, 0]
            self.iiwa.movePTPJointSpace(start_pos,[0.1])
            
            kuka_pos = self.iiwa.getEEFPos() # (in mm and rads)
            #-582.4117252266703   0.029790546497991836   545.6431178144777   3.141476197750022   -0.08725588947971301   3.141521033243545

            # Start servo for soft realtime control
            rospy.set_param('bias_ready', True)
            self.iiwa.realTime_startDirectServoCartesian()      
            time.sleep(1)

            pub = rospy.Publisher('/kuka_joints', Float32MultiArray, queue_size=1)
            sub = rospy.Subscriber('/v_kuka_in', TwistStamped, self.callback, queue_size=1)

            rospy.set_param('lin_user_scale', 50)
            rospy.set_param('rot_user_scale', 50)

            t_0 = self.getSecs()
            cycle_counter = 0
            self.send_msg_ = False
            r = rospy.Rate(rate_hz)
            print('Haption: Hold stick vertical and hold grey footswitch to move')
            print('Haption: Press left button to toggle the gripper')

            while not rospy.is_shutdown():
                if self.send_msg_:
                    lin_user_scale = rospy.get_param('/lin_user_scale')
                    rot_user_scale = rospy.get_param('/rot_user_scale')

                    for i in range(3):
                        cmd[i] = self.vel[i]*timestep*5000  *(lin_user_scale/100) + kuka_pos[i]    # in mm
                        ## Convert Kuka Rz*Ry'*Rz'' to quaternion, Q1
                        Q1 = self.R2Q(kuka_pos[3], kuka_pos[4], kuka_pos[5]) # [w,  x*i,  y*j,  z*k]
                        

                        ## apply angular velocity over a timestep, dt (vel is in zyx)
                        W  = [0, self.vel[5], self.vel[4], self.vel[3]] # [0, wx*i, wy*j, wz*k]
                        Q2 = [1, 0, 0, 0]				# [0, wx*i, wy*j, wz*k]

                        # Calculate Using Q2 = Q1+0.5*dt*W*Q1 (derived from: dQ(t)/dt = 0.5*W(t)*Q(t) )
                        WQ1 = self.QMult(W, Q1)
                        for i in range(4):
                                Q2[i] = Q1[i] + 0.5 * timestep * WQ1[i] *5

                        # Convert Q2 back to Rz*Ry'*Rz''
                        angles = self.Q2R(Q2[0], Q2[1], Q2[2], Q2[3])

                        cmd[3] = angles[0]
                        cmd[4] = angles[1]
                        cmd[5] = angles[2]

                        #cmd[i+3] = self.vel[i+3]*timestep*10 *(rot_user_scale/100) + kuka_pos[i+3]  # in rads *20
                        
                    if (self.getSecs()-t_0)>timestep:
                        #intrinsic_latency = rospy.Time.now() - self.timestamp           # To check system latency
                        #print(intrinsic_latency.to_sec())
                        kuka_pos = self.iiwa.sendEEfPositionGetActualEEFpos(cmd)        # Send position command
                        self.send_msg_ = False
                        t_0=self.getSecs()
                        cycle_counter=cycle_counter+1
                        
                kuka_joints = self.iiwa.getJointsPos()
                kuka_joints_msg.data = kuka_joints
                pub.publish(kuka_joints_msg)                           # Publish current joint positions
                r.sleep()
                
            totalT= self.getSecs()-t0;
            #self.iiwa.realTime_stopDirectServoCartesian()               # Stop direct servo
            
        except:
            if rospy.is_shutdown():
                print('Kuka Arm: ROS closed - stopping control')
            else:
                print('Kuka Arm: An error occured')

        try:
            print('Kuka Arm: Stopping servo')
            self.iiwa.realTime_stopDirectServoCartesian()               # Stop direct servo
        except:
            print('Kuka Arm: Error stopping servo')
            pass

        time.sleep(1)
        self.iiwa.realTime_stopDirectServoCartesian()
        self.disconnect_from_iiwa()                                 # Disconnect
        
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

    def callback(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z
        wx = msg.twist.angular.x
        wy = msg.twist.angular.y
        wz = msg.twist.angular.z
        self.timestamp = msg.header.stamp
        
        self.vel = [vx, vy, vz, wz, wy, wx]      # (in mm and rads) (KUKA ABC angles are in Z, Y, X order)
        

        self.send_msg_ = True

    # returns the elapsed seconds since the start of the program
    def getSecs(self):
       dt = datetime.now() - self.start_time
       secs = (dt.days * 24 * 60 * 60 + dt.seconds)  + dt.microseconds / 1000000.0
       return secs


    def QMult(self, QA, QB):
        a = QA[0] # wA
        b = QA[1] # xA
        c = QA[2] # yA
        d = QA[3] # zA

        e = QB[0] # wB
        f = QB[1] # xB
        g = QB[2] # yB
        h = QB[3] # zB

        w = a*e - b*f - c*g - d*h
        x = b*e + a*f + c*h - d*g
        y = a*g - b*h + c*e + d*f
        z = a*h + b*g - c*f + d*e

        return [w, x, y, z]

    def R2Q(self, yaw, pitch, roll):
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
    
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
    
        return [qw, qx, qy, qz]
        
    def Q2R(self, qw, qx, qy, qz):
        # Rx''
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = atan2(sinr_cosp, cosr_cosp)

        # Ry'
        sinp = 2 * (qw * qy - qz * qx)
        if (abs(sinp) >= 1):
            pitch = pi/2 * sinp/abs(sinp) # use 90 degrees if out of range
        else:
            pitch = asin(sinp)

        # Rz
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = atan2(siny_cosp, cosy_cosp)

        return [yaw, pitch, roll]

    def BaselineAxia(self):
        code = 0
        while code != 200:
            get_url = urllib.request.urlopen('http://192.168.1.2/rundata.cgi?cmd=setuserbias')
            code = get_url.getcode()
    
if __name__ == '__main__':
    rospy.init_node('KUKA', anonymous=False)
    KukaControl()

