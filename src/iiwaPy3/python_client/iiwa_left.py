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
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import keyboard

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
        kuka_pos_msg = TransformStamped()
        cmd = [None, None, None, None, None, None]
        kuka_pos = [None, None, None, None, None, None]

        try:        
            # Move to home pos
            print('Kuka Arm: Moving to home position')
            start_pos = [5.18*pi/180, -33.76*pi/180, 0.00, 110.10*pi/180, -0.01*pi/180, -36.14*pi/180, 5.17*pi/180]
            self.iiwa.movePTPJointSpace(start_pos,[0.1])
            
            #print('Kuka Arm: Moving to task start position - Pen')
            #self.iiwa.movePTPJointSpace(start_pos,[0.2])
            #start_pos = [5.27*pi/180, -37.26*pi/180, -0.01*pi/180, 110.74*pi/180, 0.01*pi/180, -31.94*pi/180, 5.26*pi/180]
            #self.iiwa.movePTPJointSpace(start_pos,[0.1])

            kuka_start_pos = self.iiwa.getEEFPos() # (in xyz mm and xyz rads)
            kuka_pos = self.iiwa.getEEFPos()
            
            # Start servo for soft realtime control
            self.iiwa.realTime_startDirectServoCartesian()      
            time.sleep(1)

            jpos_pub = rospy.Publisher('/kuka_joints', Float32MultiArray, queue_size=1)
            pos_pub = rospy.Publisher('/kuka_pos', TransformStamped, queue_size=1)
            sub = rospy.Subscriber('/LeftHandPose', TransformStamped, self.callback, queue_size=1)

            t_0 = self.getSecs()
            self.started_ = False
            r = rospy.Rate(rate_hz)
            print('Rest hand(s) on table to start control')

            self.polaris_bias = [30, 0, -1450] # x is towards the user, y is to the left, z is upwards

            while not rospy.is_shutdown():
                # Wait for start signal

                if not self.started_:
                    if keyboard.is_pressed("space"):
                        self.polaris_bias = [self.pos_cmd[0], self.pos_cmd[1], self.pos_cmd[2]]
                        self.started_ = True
                else:
                    if self.send_msg_:
                        cmd[0] = self.pos_cmd[0] + kuka_start_pos[0]    # in mm
                        cmd[1] = self.pos_cmd[1] + kuka_start_pos[1]    # in mm
                        cmd[2] = self.pos_cmd[2] + kuka_start_pos[2]    # in mm
                        cmd[3] = 0#self.pos_cmd[3] +  0*pi/180
                        cmd[4] = 0#self.pos_cmd[4] +-90*pi/180
                        cmd[5] = 0#self.pos_cmd[5] +  0*pi/180

                        kuka_pos = self.iiwa.sendEEfPositionGetActualEEFpos(cmd)        # Send position command
                        kuka_joints = self.iiwa.getJointsPos()
                        self.send_msg_ = False
                        
                        t_0 = self.getSecs()

                # Publish current kuka positions in joint and cartesian space
                t_now = rospy.get_rostime()
                t_now_secs = rospy.get_time()
                
                kuka_joints_msg.data = [kuka_joints, t_now_secs]
                jpos_pub.publish(kuka_joints_msg)                           

                kuka_quat = self.R2Q(kuka_pos[5], kuka_pos[4], kuka_pos[3])
                kuka_pos_msg.header.stamp.secs = t_now.secs
                kuka_pos_msg.header.stamp.nsecs = t_now.nsecs
                kuka_pos_msg.transform.translation.x = kuka_pos[0]
                kuka_pos_msg.transform.translation.y = kuka_pos[1]
                kuka_pos_msg.transform.translation.z = kuka_pos[2]
                kuka_pos_msg.transform.rotation.w = kuka_quat[0]
                kuka_pos_msg.transform.rotation.x = kuka_quat[1]
                kuka_pos_msg.transform.rotation.y = kuka_quat[2]
                kuka_pos_msg.transform.rotation.z = kuka_quat[3]
                pos_pub.publish(kuka_pos_msg)

                # Loop
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
        px = msg.transform.translation.x*1000 - self.polaris_bias[0]
        py = msg.transform.translation.y*1000 - self.polaris_bias[1]
        pz = msg.transform.translation.z*1000 - self.polaris_bias[2]
        qw = msg.transform.rotation.w
        qx = msg.transform.rotation.x
        qy = msg.transform.rotation.y
        qz = msg.transform.rotation.z
        self.timestamp = msg.header.stamp
        [rz, ry, rx] = self.Q2R(qw, qx, qy, qz)
        self.pos_cmd = [px, py, pz, rz, ry, rx]      # (in mm and rads) (KUKA ABC angles are in Z, Y, X order)
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
    
if __name__ == '__main__':
    rospy.init_node('KUKA', anonymous=True)
    try:
        KukaControl()
    except rospy.ROSInterruptException:
        pass
