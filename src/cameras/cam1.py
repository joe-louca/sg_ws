#!/usr/bin/env python
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
        vid_capture = cv2.VideoCapture('/dev/video0')
        
        fps = 25.0
        width = 1280/2
        height = 720/2
        
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
        frames = []

        latency = rospy.get_param('latency')
        latency = latency/2
        
        start_time = time.time()

        # Read frames on a loop
        while not rospy.is_shutdown():
            refreshing_latency = rospy.get_param('refreshing_latency')
            if refreshing_latency:
                latency = rospy.get_param('latency')
                latency = latency/2
                rospy.set_param('refreshing_latency', False)
                start_time = time.time()
            
            # Read the frame from the camera
            success, frame = vid_capture.read()
            
            if success:
                frames.append(frame)

                if time.time() - start_time > latency:
                    delayed_frame = frames.pop(0)
                    cv2.imshow("Gripper Cam", delayed_frame)
                    pub.publish(bridge.cv2_to_imgmsg(delayed_frame, encoding="bgr8"))

                key = cv2.waitKey(1)
                if key == 27:
                    break
                # Write the frame to file
                #output.write(both)
                    
                # Display the frame
                #cv2.imshow("Camera", frame)
                #if cv2.waitKey(25) & 0xFF == ord('q'):
                #    break               

                # Publish the frame
##                pub.publish(bridge.cv2_to_imgmsg(delayed_frame, encoding="bgr8"))

            # Sleep at fps rate
            r.sleep()
            
        vid_capture.release()
        cv2.destroyAllWindows()
        #output.release()


if __name__ == '__main__':
    CAMERAS()

