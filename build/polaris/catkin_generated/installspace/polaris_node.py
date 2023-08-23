#! /user/bin/python

"""
Example showing how to initialise, configure, and communicate
with NDI Polaris, Vega, and Aurora trackers.
"""

import time
import math
import rospy
from scipy.spatial.transform import Rotation as R
import numpy as np
from sksurgerynditracker.nditracker import NDITracker
from geometry_msgs.msg import TransformStamped

def angular_velocities(q1, q2, dt): 
    # q1 = last quaternion, q2 = current quaternion.
    # Quaternions are given as [w, x, y, z]
    # Returns angular velocity [x, y, z] as rad/s
    return (2 / dt) * np.array([q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2],
                                q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1],
                                q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]])

def run():   
    rospy.init_node('sg_MJC_controller', anonymous=True)
    r = rospy.Rate(60) # Tracker collects at a constant rate of 60 Hz. This script seems to do 20Hz
    
    msgs = []
    pubs = []
    
    tool_names = ['RightHand', 'LeftHand']
    rom_location = '/home/joe/sg_ws/src/polaris/data/'
    rom_files = [rom_location+'RightHand.rom', rom_location+'LeftHand.rom', rom_location+'ReferenceMarker.rom']
    num_tools = len(tool_names)

    for i in range(num_tools):
        msgs.append(TransformStamped())
        pubs.append(rospy.Publisher('/'+tool_names[i]+'Pose', TransformStamped, queue_size=1))

    
    # Define settings for tracker model and tools
    # Tool description files (.rom) can be generated using NDI 6D Architect
    # For more information on tool characterization, see the “Polaris Tool Design Guide” and the “NDI 6D Architect User Guide”
    settings_polaris = {"tracker type": "polaris", "romfiles" : rom_files}

    # Initilise tracker object for communication with NDI trackers. 
    tracker = NDITracker(settings_polaris)

    # Tell the NDI devices to start tracking.
    tracker.start_tracking()

    # Get the port handles and tool descriptions
    tool_descriptions = tracker.get_tool_descriptions()


    # Get reference frame transformation matrices
    port_handles, timestamps, framenumbers, trackings, qualitys = tracker.get_frame()
    ref_pos = trackings[2][0]
    ref_pos_q = R.from_quat([ref_pos[1], ref_pos[2], ref_pos[3], ref_pos[0]])
    ref_pos_rot = ref_pos_q.as_matrix()
    T_pol2ref = np.array([[ref_pos_rot[0][0], ref_pos_rot[0][1], ref_pos_rot[0][2], ref_pos[4]],
                          [ref_pos_rot[1][0], ref_pos_rot[1][1], ref_pos_rot[1][2], ref_pos[5]],
                          [ref_pos_rot[2][0], ref_pos_rot[2][1], ref_pos_rot[2][2], ref_pos[6]],
                          [                0,                 0,                 0,          1]])             
    T_ref2pol =  np.linalg.inv(T_pol2ref)

    while not rospy.is_shutdown():
        # tracker.get_frame() - Gets a frame of tracking data from the NDI device.
        # port_numbers: list of port handles, one per tool
        # time_stamps: list of timestamps (cpu clock), one per tool
        #              Use the frame number, and not the host computer clock,
        #              to identify when data was collected
        # frame_numbers: list of frame numbers (tracker clock), one per tool
        # tracking: list of 4x4 tracking matrices, rotation and position, or if
        #           use_quaternions is true, a list of tracking quaternions,
        #           column 0-2 is x,y,z column 3-6 is the rotation as a quaternion.
        # tracking_quality: list of the tracking quality, one per tool.

        port_handles, timestamps, framenumbers, trackings, qualitys = tracker.get_frame()
        for i in range(num_tools):
            tool_pos = trackings[i][0]
            if not math.isnan(tool_pos[0]):
                # Convert to T matrix
                hand_pos = tool_pos
                hand_quat = R.from_quat([hand_pos[1], hand_pos[2], hand_pos[3], hand_pos[0]])
                hand_rot = hand_quat.as_matrix()
                T_pol2hand = np.array([[hand_rot[0][0], hand_rot[0][1], hand_rot[0][2], hand_pos[4]],
                                       [hand_rot[1][0], hand_rot[1][1], hand_rot[1][2], hand_pos[5]],
                                       [hand_rot[2][0], hand_rot[2][1], hand_rot[2][2], hand_pos[6]],
                                       [             0,              0,              0,           1]])
                # Calculate hand pose in reference frame
                T_ref2hand = np.dot(T_ref2pol, T_pol2hand)
                
                # Convert to quaternion in the reference
                hand_quat = R.from_matrix([[T_ref2hand[0][0], T_ref2hand[0][1], T_ref2hand[0][2]],
                                           [T_ref2hand[1][0], T_ref2hand[1][1], T_ref2hand[1][2]],
                                           [T_ref2hand[2][0], T_ref2hand[2][1], T_ref2hand[2][2]]]).as_quat()

                # Publish hand pose
                msgs[i].header.stamp = rospy.Time.now()
                msgs[i].header.frame_id = tool_names[i]
                msgs[i].transform.translation.x = T_ref2hand[0][3] 
                msgs[i].transform.translation.y = T_ref2hand[1][3] 
                msgs[i].transform.translation.z = T_ref2hand[2][3] 
                msgs[i].transform.rotation.w = tool_pos[3]
                msgs[i].transform.rotation.x = tool_pos[0]
                msgs[i].transform.rotation.y = tool_pos[1]
                msgs[i].transform.rotation.z = tool_pos[2]

                pubs[i].publish(msgs[i])

        r.sleep()

    # Tell the NDI devices to stop tracking.
    tracker.stop_tracking()
    tracker.close()

if __name__ == '__main__':
    run()
