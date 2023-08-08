#! /user/bin/python

"""
Example showing how to initialise, configure, and communicate
with NDI Polaris, Vega, and Aurora trackers.
"""

import time
import math
import rospy
from sksurgerynditracker.nditracker import NDITracker
from geometry_msgs.msg import TransformStamped

def run():
    """Demonstration program

    Example showing how to initialise, configure, and communicate
    with NDI Polaris, Vega, and Aurora trackers.
    Configuration is by python dictionaries, edit as necessary.

    Dictionaries for other systems:

    settings_polaris = {"tracker type": "polaris",
    "romfiles" : ["../data/8700339.rom"]}

    settings_aurora = {
        "tracker type": "aurora",
        "ports to probe": 2,
        "verbose": True,
    }

    settings_dummy = {"tracker type": "dummy",}

    """

    
    rospy.init_node('sg_MJC_controller', anonymous=True)
    r = rospy.Rate(60) # Tracker collects at a constant rate of 60 Hz. This script seems to do 20Hz
    
    msgs = []
    pubs = []
    
    tool_names = ['Wrist']
    rom_location = '/home/joe/sg_ws/src/polaris/data/'
    rom_files = [rom_location+'RightHand.rom']
    num_tools = len(rom_files)

    for i in range(num_tools):
        msgs.append(TransformStamped())
        pubs.append(rospy.Publisher('/'+tool_names[i]+'Pose', TransformStamped, queue_size=1))

    
    # Define settings for tracker model and tools
    # Tool description files (.rom) can be generated using NDI 6D Architect
    # For more information on tool characterization, see the “Polaris Tool Design Guide” and the “NDI 6D Architect User Guide”
    settings_polaris = {"tracker type": "polaris", "romfiles" : [rom_location+"RightHand.rom"]}

    # Initilise tracker object for communication with NDI trackers. 
    tracker = NDITracker(settings_polaris)

    # Tell the NDI devices to start tracking.
    tracker.start_tracking()

    # Get the port handles and tool descriptions
    tool_descriptions = tracker.get_tool_descriptions()

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
            tool_pos = trackings[i]
            if not math.isnan(tool_pos[i][0]):
                print(tool_pos)
                msgs[i].header.stamp = rospy.Time.now()
                msgs[i].header.frame_id = tool_names[i]
                msgs[i].transform.translation.x = tool_pos[i][0]
                msgs[i].transform.translation.y = tool_pos[i][1]
                msgs[i].transform.translation.z = tool_pos[i][2]
                msgs[i].transform.rotation.w = tool_pos[i][3]
                msgs[i].transform.rotation.x = tool_pos[i][4]
                msgs[i].transform.rotation.y = tool_pos[i][5]
                msgs[i].transform.rotation.z = tool_pos[i][6]

                pubs[i].publish(msgs[i])
            
        r.sleep()

    # Tell the NDI devices to stop tracking.
    tracker.stop_tracking()
    tracker.close()

if __name__ == '__main__':
    run()
