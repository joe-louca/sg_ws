#!/usr/bin/env python3

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from std_msgs.msg import Float32
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep

def gripper_cmd_callback(msg):
    global gripper_cmd
    global cmd_ready
    gripper_cmd = msg.data
    cmd_ready = True

def publisher():
    global gripper_cmd
    global cmd_ready
    gripper_max = 50
    
    rospy.init_node('Robotiq2FGripperSimpleController')    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
    sub = rospy.Subscriber('/Delayed_TPDistance', Float32, gripper_cmd_callback)
    rate_hz = 100
    
    #rate_hz = rospy.get_param('rate_hz')
    
    command = outputMsg.Robotiq2FGripper_robot_output();
    r = rospy.Rate(rate_hz)

    # Send reset command
    command.rACT = 0  # activate
    command.rGTO = 0  # go to action
    command.rATR = 0  # Reset??
    command.rPR = 0 # closed
    command.rSP = 0 # speed
    command.rFR = 0 #force
    pub.publish(command)
    sleep(0.1)
    
    # Send activate command - activate
    command.rACT = 1  # activate
    command.rGTO = 1  # go to action
    command.rATR = 0  # Reset??
    command.rPR = 130 # open
    command.rSP = 100 # speed
    command.rFR = 100 # force
    pub.publish(command)
    sleep(0.1)

    i=0
    smooth_num = 1
    gripper_smooth_arr = [None] * smooth_num
    full = False
    last_pos = 0
    
    cmd_ready = False
    while not rospy.is_shutdown():       
        try:
            if cmd_ready:
                TESTING = rospy.get_param('testing')
                if not TESTING:
                    gripper_smooth_arr[i] = gripper_cmd

                    i += 1
                    if i == smooth_num:
                        full = True
                        i = 0

                    if full:
                        gripper_smoothed = sum(gripper_smooth_arr) / smooth_num
                        
                        # build command msg
                        if gripper_smoothed > gripper_max:
                            gripper_pos = 0 # open
                        elif gripper_smoothed < 0:
                            gripper_pos = 255 # closed
                        else:
                            gripper_pos = 255+int(-255/gripper_max*gripper_smoothed)
                        
                        if (gripper_pos == last_pos+1) or (gripper_pos == last_pos-1):
                            gripper_pos = last_pos
                        
                        command.rACT = 1  # activate
                        command.rGTO = 1  # go to action
                        command.rATR = 0  # Reset??
                        command.rPR = gripper_pos # 255 open, 0 closed
                        command.rSP = 50 #150 # max speed
                        command.rFR = 200 # max force
                        # publish to gripper        
                        pub.publish(command)

                        last_pos = gripper_pos
                    
                cmd_ready = False

                
        except:
            pass
        r.sleep()
                        

if __name__ == '__main__':
    publisher()
