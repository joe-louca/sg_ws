import mujoco
import mujoco_viewer
import time
import math
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped

import numpy as np

class SG_CONTROL:
    def callback_rh(self, msg):       
        # Thumb
        self.pos_rh[0] = msg.transform.translation.x/1000   # x
        self.pos_rh[1] = msg.transform.translation.y/1000   # y 
        self.pos_rh[2] = msg.transform.translation.z/1000-0.125   # z
        self.pos_rh[3] = msg.transform.rotation.x
        self.pos_rh[4] = msg.transform.rotation.y
        self.pos_rh[5] = msg.transform.rotation.z
        self.pos_rh[6] = msg.transform.rotation.w
        self.received_rh_cmd_ = True
        
    def callback_lh(self, msg):       
        # Thumb
        self.pos_lh[0] = msg.transform.translation.x/1000   # x
        self.pos_lh[1] = msg.transform.translation.y/1000   # y 
        self.pos_lh[2] = msg.transform.translation.z/1000-0.125  # z
        self.pos_lh[3] = msg.transform.rotation.x
        self.pos_lh[4] = msg.transform.rotation.y
        self.pos_lh[5] = msg.transform.rotation.z
        self.pos_lh[6] = msg.transform.rotation.w
        self.received_lh_cmd_ = True
        
    def callback_sg(self, msg):
        self.thumb2pointer_distance = msg.data
        self.received_sg_cmd_ = True

    def get_sensor_sensordata(self, body_name):
        return self.data.sensordata
    
    def __init__(self):
        path_to_models = '/home/joe/sg_ws/src/mujoco_menagerie/'
        self.deg2rad = math.pi/180
        self.pos_rh = np.zeros(7)
        self.pos_lh = np.zeros(7)
        self.received_rh_cmd_ = False
        self.received_lh_cmd_ = False
        self.received_sg_cmd_ = False

        sg_max_distance = 100
        
        rospy.init_node('sg_MJC_controller', anonymous=True)
        t0 = rospy.get_time()
        pub = rospy.Publisher('/FingerContacts', Float32MultiArray, queue_size=1)
        sub = rospy.Subscriber('/RightHandPose', TransformStamped, self.callback_rh, queue_size=1)
        sub = rospy.Subscriber('/LeftHandPose', TransformStamped, self.callback_lh, queue_size=1)
        sub = rospy.Subscriber('/TPDistance', Float32, self.callback_sg, queue_size=1)
        rate_hz = 200
        r = rospy.Rate(rate_hz)

        contact_msg = Float32MultiArray()
        self.grip_cmd_ = False

        model = mujoco.MjModel.from_xml_path(path_to_models + 'myscenes/2f85_left.xml')
        data = mujoco.MjData(model)
        contact_geom_ids = [model.geom('left_pad1').id, model.geom('right_pad1').id]

        # create the viewer object
        viewer = mujoco_viewer.MujocoViewer(model, data, hide_menus=False)
        viewer.cam.lookat[0] = 0.3
        viewer.cam.lookat[1] = -0.0
        viewer.cam.lookat[2] = 0.2
        viewer.cam.distance = 1.35
        viewer.cam.elevation = -20
        viewer.cam.azimuth = +0
        viewer._paused = True
        
        # simulate and render
        frm = 0
        while not rospy.is_shutdown():
            if self.received_lh_cmd_:
                # Update position of the wrist base
                data.qpos[0] = self.pos_lh[0] # x
                data.qpos[1] = self.pos_lh[1] # y
                data.qpos[2] = self.pos_lh[2] # z
                data.qpos[3] = self.pos_lh[3] # ball
                data.qpos[4] = self.pos_lh[4] # ball
                data.qpos[5] = self.pos_lh[5] # ball
                data.qpos[6] = self.pos_lh[6] # ball
                
            if self.received_sg_cmd_:
                # Set grip position: 0=open, 255=closed
                if ((sg_max_distance - self.thumb2pointer_distance) < 0):
                    grip_pos = 0
                else:
                    grip_pos = ((sg_max_distance - self.thumb2pointer_distance) / sg_max_distance) * 255
                data.ctrl[3] = grip_pos

            # Read contacts
            #geom ids: left pad = 13, right_pad = 25, box = 29
            contact_msg.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            for i in range(data.ncon):
                contact = data.contact[i]
                dist = contact.dist         # -ve values are penetration
                friction = contact.friction # 5x1 (tangent1, 2, spin, roll1, 2)
                if dist < -0.00001:
                    if contact.geom1 == contact_geom_ids[0]: # right
                        contact_msg.data[0] = 100
                        contact_msg.data[5] = 100
                    elif contact.geom1 == contact_geom_ids[1]: # left
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
