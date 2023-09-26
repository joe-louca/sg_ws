# -*- coding: utf-8 -*-

from iiwaPy3 import iiwaPy3
import time
import rospy
import os
import cv2
from math import pi
import roslaunch

class KukaControl:
    def __init__(self):
        #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #roslaunch.configure_logging(uuid)
        #launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/joe/sg_ws/src/launch/restart.launch"])

        raw_gui_img = cv2.imread('/home/joe/RobotiqSideProfile.png')
        scoreboards = []
        for i in range(22):
            raw_gui_img = cv2.imread('/home/joe/sg_ws/src/Scoreboards/Scoreboard_' + str(i) + '.png')
            scale_percent = 25 # percent of original size
            width = int(raw_gui_img.shape[1] * scale_percent / 100)
            height = int(raw_gui_img.shape[0] * scale_percent / 100)
            dim = (width, height)
            scoreboard_img = cv2.resize(raw_gui_img, dim, interpolation = cv2.INTER_AREA)
            scoreboards.append(scoreboard_img)
            
        scoreboard_img = scoreboards[0]

        #self.start_time = datetime.now()                            # Start time for getSecs()
        self.timestamp = rospy.Time.now()
        
        # Setup connection to iiwa
        self.IP_of_robot = '172.31.1.148'        
        self.connection_state = False
        self.connect_to_iiwa()


        # Define start and move pos
        start_pos = [-1.12*pi/180, -25.80*pi/180, -0.0*pi/180, 78.06*pi/180, 0.00*pi/180, -76.12*pi/180, (-1.20+60)*pi/180]
        end_pos = [-1.12*pi/180, -24.53*pi/180, -0.0*pi/180, 72.77*pi/180, 0.0*pi/180, -82.70*pi/180, (-1.20+60)*pi/180]

        # Move to home pos
        print('Moving to start position...')
        self.iiwa.movePTPJointSpace(start_pos,[0.1])
        trial_num = 0
        space = 32
        esc = 27
        enter = 13
        finished = False
        
        refreshing_latency = False
        rospy.set_param('testing', False)
        rospy.set_param('refreshing_latency', refreshing_latency)
        print('Ready.')

        first_loop = True
        score_count = 0
        rospy.set_param('begin', False)
        
        while not finished:
            refreshing_latency = rospy.get_param('refreshing_latency')
            if not refreshing_latency:
                cv2.imshow(" ",scoreboard_img)
                k = cv2.waitKey(1)
                # get keyboard input
                if k == -1:
                    continue
                elif k == space:
                    print('Paused control for test')
                    rospy.set_param('testing', True)
                    #launch.shutdown()
                    #os.system('rosnode kill /DelayTPDistance')

                    print('Testing...')
                    self.iiwa.movePTPJointSpace(end_pos,[0.1])
                    time.sleep(0.1)
                    self.iiwa.movePTPJointSpace(start_pos,[0.1])
                    print('Test complete.')

                    
                    # If 3 grasps complete, increase the latency
                    trial_num += 1
                    if trial_num % 3 == 0:
                        refreshing_latency = True
                        latency = (trial_num / 3)*0.5
                        rospy.set_param('latency', latency)
                        rospy.set_param('refresh_latency', refreshing_latency)
                        print('Delay is now ' + str(round(latency,1)) + ' seconds')

                    # Wait for me to press enter
                    success = False
                    while not success:
                        cv2.imshow(" ",scoreboard_img)
                        k = cv2.waitKey(1)
                        if k == -1:
                            continue
                        elif k == enter:
                            success = True
                        elif k == esc:
                            success = True
                            finished = True
                            print('Level reached: ' + str(score_count))
                    
                    rospy.set_param('testing', False)
                    print('Continue control')
                    k = -1
                    
                    score_count += 1
                    scoreboard_img = scoreboards[score_count]
                    continue
                    
                elif k == esc:
                    finished = True
                    print('Level reached: ' + str(score_count))

                elif k == enter and first_loop:
                    rospy.set_param('begin', True)
                
        ## Disconnect
        cv2.destroyAllWindows()
        self.disconnect_from_iiwa()                                 # Disconnect
        
    def connect_to_iiwa(self):
        # Check if already connected to the robot
        if self.connection_state:
            print("Kuka Arm: Already connected to the robot on IP: " + self.IP_of_robot)
            return

        # If the program made it to here, then there is no connection yet so try to connect
        print("Kuka Arm: Connecting to robot at ip: " + self.IP_of_robot)
        try:
            self.iiwa = iiwaPy3(self.IP_of_robot)
            self.connection_state = True
            print("Kuka Arm: Connection established successfully")
            return
            
        except:
            print("Kuka Arm: Error, could not connect at the specified IP")
            return            

    def disconnect_from_iiwa(self):
        # Check if there is an active connection
        print("Kuka Arm: Disconnecting from robot")
        if self.connection_state == False:
            print("Kuka Arm: Already offline")
            return

        # If made it to here, then there is an active connection so try to disconnect
        try:
            self.iiwa.close()
            self.connection_state = False
            print("Kuka Arm: Disconnected successfully")
            
        except:
            print("Kuka Arm: Error could not disconnect")
            return


    # returns the elapsed seconds since the start of the program
    def getSecs(self):
       dt = datetime.now() - self.start_time
       secs = (dt.days * 24 * 60 * 60 + dt.seconds)  + dt.microseconds / 1000000.0
       return secs

if __name__ == '__main__':
    rospy.init_node('KUKA', anonymous=False)
    try:
        KukaControl()
    except rospy.ROSInterruptException:
        pass
