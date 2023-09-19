#!/usr/bin/env python3
import rospy
import numpy as np
import os
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CAMERAS:
    def __init__(self):
        rospy.init_node('CameraNode', anonymous=True)
        pub = rospy.Publisher('/Camera', Image, queue_size=1)
        img_msg = Image()
        rate_hz = 25
        
        # Set up video captures
        vid_capture = cv2.VideoCapture('/dev/video4')
        
        fps = 25.0
        width = 1280
        height = 720
        
        vid_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        vid_capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        vid_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        vid_capture.set(cv2.CAP_PROP_FPS,fps)


        # Set up video recording
##        path =  '~/UserTrialData/taskcam.mp4'
##        vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
##        output = cv2.VideoWriter(os.path.expanduser(path), vid_cod, fps, (width,height))


        bridge = CvBridge()
        
        self.t0 = rospy.get_time()
        r = rospy.Rate(rate_hz)
        
        # Read frames on a loop
        while not rospy.is_shutdown():
            # Read the frame from the camera
            success, frame = vid_capture.read()
            
            if success:                
                # Write the frame to file
                #output.write(both)
                    
                # Display the frame
                #cv2.imshow("Camera", frame)
                #if cv2.waitKey(25) & 0xFF == ord('q'):
                #    break               

                # Publish the frame
                pub.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

            # Sleep at fps rate
            r.sleep()
            
        vid_capture.release()
        cv2.destroyAllWindows()
        #output.release()


if __name__ == '__main__':
    CAMERAS()
