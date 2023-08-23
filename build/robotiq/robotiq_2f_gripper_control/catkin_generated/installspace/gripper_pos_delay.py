#!/usr/bin/env python3

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from std_msgs.msg import Int16
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

class GRIPPER_LISTENER:
    def add_delay(self, added_row, delayed_tbl):
        self.latency = rospy.get_param('latency')
        delayed_tbl.insert(0, added_row)                                # Add new row to table
        row_len = len(added_row)-1
        tbl_len = len(delayed_tbl)                              
        retrieved_row = []
        retrieved = False
        elapsed_time = rospy.get_time()
        
        for i in range(tbl_len):                                        # For each row
            if elapsed_time > delayed_tbl[i][row_len] + self.latency:   # If row is old enough
                retrieved_row = delayed_tbl[i]                          # Get this row
                retrieved_row = retrieved_row[:(row_len)]             # Remove timestamp
                delayed_tbl = delayed_tbl[:(-tbl_len+i-1)]              # Remove old rows
                retrieved = True                                        # Update marker
                break
        
        return retrieved_row, retrieved, delayed_tbl
    
    def pub_delayed_gripper(self, status):
        t = rospy.get_time()
        timestamped_gripper_pos = [status.gPR, t]
        delayed_gripper_pos, retrieved, self.delayed_gripper_pos = self.add_delay(timestamped_gripper_pos, self.delayed_gripper_pos)

        # If a delayed position retrieved, then publish to 'delayed_gripper_pos'
        if retrieved:
            m = Int16()
            m.data = delayed_gripper_pos[0]
            self.pub.publish(m)
            
    def Robotiq2FGripperStatusListener(self):
        rospy.init_node('Robotiq2FGripperReceiveDelay')
        rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.pub_delayed_gripper)
        rospy.spin()

    def __init__(self):
        self.latency = rospy.get_param('latency')
        self.delayed_gripper_pos = []
        self.pub = rospy.Publisher('delayed_gripper_pos', Int16, queue_size=1)
        self.Robotiq2FGripperStatusListener()
        
if __name__ == '__main__':
    GRIPPER_LISTENER()
