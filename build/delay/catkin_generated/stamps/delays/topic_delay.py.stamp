#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
import numpy as np

class TOPIC_DELAY():
    def callback(self, msg):

        # Timestamp
        t = rospy.get_time() - self.t0
        if not np.any(self.delayed_tbl):
            self.delayed_tbl = np.array([msg, t])
        else:
            timestamped_msg = np.array([msg, t])
            self.delayed_tbl = np.vstack([self.delayed_tbl, timestamped_msg])
        t_delay = t - self.latency
        
        # Find indexes for old rows
        if self.delayed_tbl.ndim != 1:
            old_rows = np.where(self.delayed_tbl[:,-1] < t_delay)
            
            # If a delayed row has been retrieved
            if np.any(old_rows):
                # Get the latest one as a message
                self.delayed_msg = self.delayed_tbl[old_rows[0][-1]][0]
                self.delayed_msg_t = self.delayed_tbl[old_rows[0][-1]][1]
                self.checker = True
                
                # Delete all but the latest rows
                self.delayed_tbl = self.delayed_tbl[1:,:] # Not sure why this line works but looks like it does...


    def __init__(self, topic_name):
        rospy.init_node(topic_name+'_DelayNode', anonymous=True)
        self.checker = False

        # Prepare the publisher and subscriber, based on the topic        
        if topic_name == 'FingerContacts' or topic_name == 'FingerJointAngles':
            self.delayed_msg = Float32MultiArray()
            pub = rospy.Publisher('/Delayed_'+topic_name, Float32MultiArray, queue_size=1)
            sub = rospy.Subscriber('/'+topic_name, Float32MultiArray, self.callback, queue_size=1)
            self.rate_hz = 200
        elif topic_name == 'Camera':
            self.delayed_msg = Image()
            pub = rospy.Publisher('/Delayed_'+topic_name, Image, queue_size=1)
            sub = rospy.Subscriber('/'+topic_name, Image, self.callback, queue_size=1)
            self.rate_hz = 25
        else: #TPDistance
            self.delayed_msg = Float32()
            pub = rospy.Publisher('/Delayed_'+topic_name, Float32, queue_size=1)
            sub = rospy.Subscriber('/'+topic_name, Float32, self.callback, queue_size=1)
            self.rate_hz = 200

        # Get some parameters
        self.latency = rospy.get_param('latency')
        self.delayed_tbl = np.array([None, None])
    
        self.t0 = rospy.get_time()
        r = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            if self.checker:
                pub.publish(self.delayed_msg)
                self.checker = False
                self.latency = rospy.get_param('latency')
            r.sleep()



if __name__ == '__main__':
    delay_node = TOPIC_DELAY(sys.argv[1])


