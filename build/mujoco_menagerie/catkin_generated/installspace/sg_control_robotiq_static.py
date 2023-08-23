import mujoco
import mujoco_viewer
import time
import math
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np

class SG_CONTROL:
    def callback(self, msg):
        self.thumb2pointer_distance = msg.data
        self.grip_cmd_ = True

    def get_sensor_sensordata(self, body_name):
        return self.data.sensordata
    
    def __init__(self):
        path_to_models = '/home/joe/sg_ws/src/mujoco_menagerie/'
        
        rospy.init_node('sg_MJC_controller', anonymous=True)
        t0 = rospy.get_time()
        pub = rospy.Publisher('/FingerContacts', Float32MultiArray, queue_size=1)
        sub = rospy.Subscriber('/TPDistance', Float32, self.callback, queue_size=1)
        rate_hz = 200
        r = rospy.Rate(rate_hz)

        contact_msg = Float32MultiArray()
        self.grip_cmd_ = False

        sg_max_distance = 100

        #model = mujoco.MjModel.from_xml_path('shadow_hand/scene_left.xml')
        model = mujoco.MjModel.from_xml_path(path_to_models + 'robotiq_2f85/scene.xml')
        data = mujoco.MjData(model)

        # create the viewer object
        viewer = mujoco_viewer.MujocoViewer(model, data, hide_menus=False)
        viewer.cam.distance = 1.35
        viewer.cam.elevation = -30
        viewer.cam.azimuth = +60
        viewer._paused = True
        
        # simulate and render
        frm = 0
        while not rospy.is_shutdown():
            if self.grip_cmd_:
                # Set grip position: 0=open, 255=closed
                grip_pos = ((sg_max_distance - self.thumb2pointer_distance) / sg_max_distance) * 255
                #grip_pos = math.cos(frm / 100) * 255
                data.ctrl[0] = grip_pos

            # Read contacts
            #geom ids: left pad = 13, right_pad = 25, box = 29
            contact_msg.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            for i in range(data.ncon):
                contact = data.contact[i]
                dist = contact.dist         # -ve values are penetration
                friction = contact.friction # 5x1 (tangent1, 2, spin, roll1, 2)
                if dist < -0.00001:
                    if contact.geom1 == 13:
                        contact_msg.data[0] = 100
                        contact_msg.data[5] = 100
                    elif contact.geom1 == 25:
                        contact_msg.data[1] = 100
                        contact_msg.data[6] = 100

            contact_msg.data[10] = rospy.get_time() - t0
            pub.publish(contact_msg)        # [Left Buzz, Left Force, Right Buzz, Right Force, Time]
            
            if viewer.is_alive:
                mujoco.mj_step(model, data)
                viewer.render()
            else:
                break

            frm += 1
            r.sleep

        # close
        viewer.close()

if __name__ == '__main__':
    foo = SG_CONTROL()
