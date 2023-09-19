import cv2
import matplotlib
import numpy
import rospy
from std_msgs import Float32MultiArray

class VisualFB():
    def callback(self, msg):
        self.F_L = msg.data[0]
        self.F_R = msg.data[1]

    def __init__(self):
        radius = 20
        thickness = 2
        center_coordinates_L = (120, 50) # x, y
        center_coordinates_R = (500, 50) # x, y
        F_max = 100

        rospy.init_node('VisualFeedbackNode', anonymous=True)
        self.checker = False
        self.F_L = 0
        self.F_R = 0
        
        # Prepare the publisher and subscriber, based on the topic        
        sub = rospy.Subscriber('/FingerContacts', Float32MultiArray, self.callback, queue_size=1)

        self.rate_hz = 200
        r = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            #if force = 0, colour is white (255,255,255), if force = max, colour is red
            img=cv2.imread("robotiq.png")
            color_L = (round((-(F_L-F_max)/F_max)*255), round((-(F_L-F_max)/F_max)*255), 255) # BGR
            cv2.circle(img, center_coordinates_L, radius, color_L, thickness)

            color_R = (round((-(F_R-F_max)/F_max)*255), round((-(F_R-F_max)/F_max)*255), 255) # BGR
            cv2.circle(img, center_coordinates_R, radius, color_R, thickness)

            cv2.imshow("Finger Contacts",img)
            k=cv2.waitKey(0) & 0XFF
            if k== 27 :
                break
            r.sleep()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    VisualFB()
