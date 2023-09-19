#!/usr/bin/env python3
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from std_msgs.msg import Float32MultiArray
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
import numpy as np

class ROBOTIQ_FORCE_LISTENER:
    def callback(self, msg):
##        t = rospy.get_time() - self.t0
##        force = 0
##        obj = msg.gOBJ
##        if obj == 2:
##            force = 100
##        self.contact_msg.data = [force, force, 0, 0, 0, 0, 0, 0, 0, 0, t]
##        self.msg_received = True

        obj = msg.gOBJ
        if obj == 2:
            obj = 2
        else:
            obj = 0
        self.current_smooth[self.current_smooth_count] = obj
        self.current_smooth_count += 1
        if self.current_smooth_count >= self.current_smooth_max:
            self.current_smooth_count = 0
            self.current_full = True

        if self.current_full:
            obj = np.average(self.current_smooth)
                    
            t = rospy.get_time() - self.t0
            force = 0
            print(obj)
            if obj > 0:
                force = 100
            self.contact_msg.data = [force, force, 0, 0, 0, force, force, 0, 0, 0, t]
            self.msg_received = True

        
    def __init__(self):
        self.current_smooth_count = 0
        self.current_smooth_max = 4
        self.current_smooth = np.zeros(self.current_smooth_max)
        self.current_full = False
        rospy.init_node('RobotiqForceListener')
        self.t0 = rospy.get_time()
        self.contact_msg = Float32MultiArray()
        self.msg_received = False

        rospy.Subscriber('/Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, self.callback, queue_size=1)
        pub = rospy.Publisher('/FingerContacts', Float32MultiArray, queue_size=1)
        self.rate_hz = 200
        r = rospy.Rate(self.rate_hz)
        
        while not rospy.is_shutdown():
            if self.msg_received:
                pub.publish(self.contact_msg)
                self.msg_received = False
            r.sleep()

if __name__ == '__main__':
    ROBOTIQ_FORCE_LISTENER()
