# -*- coding: utf-8 -*-

from iiwaPy3 import iiwaPy3
import time
import rospy
import os
import cv2
from math import pi
import roslaunch

#uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#roslaunch.configure_logging(uuid)
#launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/joe/sg_ws/src/launch/restart.launch"])

raw_gui_img = cv2.imread('/home/joe/RobotiqSideProfile.png')
finished = False
while not finished:
        cv2.imshow(" ",raw_gui_img)
        k = cv2.waitKey(1)
        print(k)
        if k == 27:
            break
