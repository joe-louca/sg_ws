# -*- coding: utf-8 -*-

from iiwaPy3 import iiwaPy3
import time
import rospy
import os
import cv2

class KukaControl:
    def __init__(self):
        raw_gui_img = cv2.imread('/home/joe/RobotiqSideProfile.png')
        
##        self.start_time = datetime.now()                            # Start time for getSecs()
##        self.timestamp = rospy.Time.now()
        
        # Setup connection to iiwa
        self.IP_of_robot = '172.31.1.148'        
        self.connection_state = False
##        self.connect_to_iiwa()


        # Define start and move pos
        start_pos = [4.27*pi/180, -42.15*pi/180, 0.02*pi/180, 91.13*pi/180, 0.01*pi/180, -46.57*pi/180, 4.26*pi/180]
        end_pos = [4.27*pi/180, -42.15*pi/180, 0.02*pi/180, 91.13*pi/180, 0.01*pi/180, (-46.57+90)*pi/180, 4.26*pi/180]

        # Move to home pos
        print('Moving to start position')
##        self.iiwa.movePTPJointSpace(start_pos,[0.1])
        trial_num = 0
        space = 32
        esc = 27
        while True:
            cv2.imshow("GUI",raw_gui_img)
            k = cv2.waitKey()

            elif k == esc:
                print('esc')
                break
            elif k == space:
                print('space')

                
            # get keyboard input
            if k == -1:
                continue
            if k == space:
                print('Paused control for test')
                os.system('rosnode kill /DelayTPDistance')

                print('Testing grasp...')
##                self.iiwa.movePTPJointSpace(end_pos,[0.1])
                time.sleep(1)
##                self.iiwa.movePTPJointSpace(start_pos,[0.1])
                print('Test complete.')
                
                # If 3 grasps complete, increase the latency
                trial_num += 1
                if trial_num % 3 == 0:
                    latency = (trial_num / 3)*0.5
##                    rospy.set_param('latency', latency)
                    print('Delay is now ' + str(round(latency,1)) + ' seconds')

                os.system('rosrun delays topic_delay TPDistance')
                print('Continue control')
                
            if k == esc:
                break
                #Finish
                
        ## Disconnect
        cv2.destroyAllWindows()
##        self.disconnect_from_iiwa()                                 # Disconnect
        
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
    #rospy.init_node('KUKA', anonymous=True)
    try:
        KukaControl()
    except rospy.ROSInterruptException:
        pass
