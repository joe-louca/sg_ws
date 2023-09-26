import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Int16
import csv
import time
import sys

class DATA():
    def callback_fc(self, msg):
        self.F_L = msg.data[0]
        self.F_R = msg.data[1]
        
    def callback_grip(self, msg):
        self.GRIP = msg.data
        
    def callback_egg(self, msg):
        self.EGG = msg.data
        
    def __init__(self):
        start_time = time.time()
        rospy.init_node('DataRecorderNode', anonymous=False)
        p = rospy.get_param('participant')
        fb = rospy.get_param('feedback')
        with open('/home/joe/sg_ws/output/data_P'+str(p)+'_FB'+str(fb)+'.csv', 'w', newline='') as f:
            # create the csv writer
            writer = csv.writer(f, quoting=csv.QUOTE_ALL)


            # write a row to the csv file
            writer.writerow(['time', 'F_left', 'F_right', 'grip', 'egg'])       

            self.F_L = 0
            self.F_R = 0
            self.GRIP = 0
            self.EGG = 0
            
            # Prepare the publisher and subscriber, based on the topic        
            sub_fc = rospy.Subscriber('/FingerContacts', Float32MultiArray, self.callback_fc, queue_size=1)
            sub_grip = rospy.Subscriber('/TPDistance', Float32, self.callback_grip, queue_size=1)
            sub_egg = rospy.Subscriber('/EggCrack', Int16, self.callback_egg, queue_size=1)
            
            # Each row = time, left_force, right_force, gripper_position, egg_status
            data = [0, 0, 0, 0, 0]

            self.rate_hz = 200
            r = rospy.Rate(self.rate_hz)

            
            while not rospy.is_shutdown():
                t = time.time() - start_time
                data = [t, self.F_L, self.F_R, self.GRIP, self.EGG]
                testing = rospy.get_param('testing')
                if testing:
                    data = [t, -999, -999, -999, -999]
                writer.writerow(data)  
                r.sleep()


if __name__ == '__main__':
    DATA()
