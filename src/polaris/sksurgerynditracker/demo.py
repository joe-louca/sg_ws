#! /user/bin/python

"""
Example showing how to initialise, configure, and communicate
with NDI Polaris, Vega, and Aurora trackers.
"""

import time
import six
from sksurgerynditracker.nditracker import NDITracker

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

    # Define settings for tracker model and tools
    # Tool description files (.rom) can be generated using NDI 6D Architect
    # For more information on tool characterization, see the “Polaris Tool Design Guide” and the “NDI 6D Architect User Guide”
    settings_polaris = {"tracker type": "polaris",
    "romfiles" : ["../data/8700339.rom", "../data/something_else.rom"]}

    # Initilise tracker object for communication with NDI trackers. 
    tracker = NDITracker(settings_polaris)

    # Tell the NDI devices to start tracking.
    tracker.start_tracking()

    # Print the port handles and tool descriptions
    print(tracker.get_tool_descriptions())

    print('Starting')
    for _ in range(20):
        # Get a frame of tracking data from the NDI device.
        # port_numbers: list of port handles, one per tool
        # time_stamps: list of timestamps (cpu clock), one per tool
        #              Use the frame number, and not the host computer clock,
        #              to identify when data was collected
        # frame_numbers: list of frame numbers (tracker clock), one per tool
        # tracking: list of 4x4 tracking matrices, rotation and position, or if
        #           use_quaternions is true, a list of tracking quaternions,
        #           column 0-2 is x,y,z column 3-6 is the rotation as a quaternion.
        # tracking_quality: list of the tracking quality, one per tool.
        print(tracker.get_frame())

        # Tracker collects at a constant rate of 60 Hz. This script seems to do 20Hz
        time.sleep(1/60)
        print('frame ' + str(_))

    # Tell the NDI devices to stop tracking.
    tracker.stop_tracking()
    tracker.close()

run()
