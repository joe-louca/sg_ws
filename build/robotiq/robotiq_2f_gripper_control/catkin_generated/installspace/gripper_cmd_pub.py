#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a 2F gripper.

This serves as an example for publishing messages on the 'Robotiq2FGripperRobotOutput' topic using the 'Robotiq2FGripper_robot_output' msg type for sending commands to a 2F gripper.
"""

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
    gripper_max = 100
    
    """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
    rospy.init_node('Robotiq2FGripperSimpleController')    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
    sub = rospy.Subscriber('/Delayed_TPDistance', Float32, gripper_cmd_callback)
    rate_hz = 200
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
    smooth_num = 10
    gripper_smooth_arr = [None] * smooth_num
    full = False
    
    cmd_ready = False
    while not rospy.is_shutdown():       
        try:
            if cmd_ready:
                gripper_smooth_arr[i] = gripper_cmd

                i += 1
                if i == smooth_num:
                    full = True
                    i = 0

                if full:
                    gripper_smoothed = sum(gripper_smooth_arr) / smooth_num
                    print(gripper_smoothed)
                    # build command msg
                    if gripper_smoothed > gripper_max:
                        gripper_pos = 255
                    elif gripper_smoothed < 0:
                        gripper_pos = 0
                    else:
                        gripper_pos = 255-int(255*gripper_smoothed/gripper_max)
                        
                    command.rACT = 1  # activate
                    command.rGTO = 1  # go to action
                    command.rATR = 0  # Reset??
                    command.rPR = gripper_pos # 255 open, 0 closed
                    command.rSP = 150 # max speed
                    command.rFR = 200 # max force
                    # publish to gripper        
                    pub.publish(command)
                    
                cmd_ready = False

                
        except:
            pass
        r.sleep()
                        

if __name__ == '__main__':
    publisher()
