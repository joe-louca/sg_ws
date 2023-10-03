import cv2
import matplotlib
import numpy
import rospy
from std_msgs.msg import Float32MultiArray
import os

class VisualFB():
    def callback(self, msg):
        self.F_L = msg.data[0]
        self.F_R = msg.data[1]

    def __init__(self):


        rospy.init_node('VisualFeedbackNode', anonymous=True)
        self.checker = False
        self.F_L = 0
        self.F_R = 0
        
        # Prepare the publisher and subscriber, based on the topic        
        sub = rospy.Subscriber('/Delayed_FingerContacts', Float32MultiArray, self.callback, queue_size=1)

        self.rate_hz = 200
        r = rospy.Rate(self.rate_hz)

        raw_img = cv2.imread('/home/joe/sg_ws/src/fingertips/src/RobotiqSideProfile.png')
        img_v = cv2.flip(raw_img, 0)
        scale_percent = 20 # percent of original size
        width = int(img_v.shape[1] * scale_percent / 100)
        height = int(img_v.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(img_v, dim, interpolation = cv2.INTER_AREA)
 
        radius = 20
        thickness = -1
        center_coordinates_L = (250, int(height-100)) # x, y
        center_coordinates_R = (width-250, int(height-100)) # x, y
        F_max = 100

        start_L = (223, 570)
        end_L = (223+25, 745)
        start_R = (605-25, 570)
        end_R = (605, 745)
        
        start_L = (110, 285)
        end_L = (110+12, 372)
        start_R = (290, 285)
        end_R = (302, 372)        
        while not rospy.is_shutdown():
            img = resized
            #if force = 0, colour is white (255,255,255), if force = max, colour is red
            color_L = (round((-(self.F_L-F_max)/F_max)*255), round((-(self.F_L-F_max)/F_max)*255), 255) # BGR
            cv2.rectangle(img, start_L, end_L, color_L, thickness)

            color_R = (round((-(self.F_R-F_max)/F_max)*255), round((-(self.F_R-F_max)/F_max)*255), 255) # BGR
            cv2.rectangle(img, start_R, end_R, color_R, thickness)

            cv2.imshow("Contacts",img)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                  break
            r.sleep()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    VisualFB()
