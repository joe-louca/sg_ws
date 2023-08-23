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
        self.pos_rh[2] = msg.transform.translation.z/1000   # z
        self.pos_rh[3] = msg.transform.rotation.x
        self.pos_rh[4] = msg.transform.rotation.y
        self.pos_rh[5] = msg.transform.rotation.z
        self.pos_rh[6] = msg.transform.rotation.w
        self.received_rh_cmd_ = True
        
    def callback_lh(self, msg):       
        # Thumb
        self.pos_lh[0] = msg.transform.translation.x/1000   # x
        self.pos_lh[1] = msg.transform.translation.y/1000   # y 
        self.pos_lh[2] = msg.transform.translation.z/1000   # z
        self.pos_lh[3] = msg.transform.rotation.x
        self.pos_lh[4] = msg.transform.rotation.y
        self.pos_lh[5] = msg.transform.rotation.z
        self.pos_lh[6] = msg.transform.rotation.w
        self.received_lh_cmd_ = True

    def callback_sg(self, msg):       
        # Thumb
        self.finger_jpos[0] = msg.data[0]   # Twist
        self.finger_jpos[1] = -msg.data[1]  # Adduct
        #self.finger_jpos[2] = msg.data[2]  # Flex1 SG
        self.finger_jpos[3] = msg.data[3]   # Flex2
        self.finger_jpos[4] = msg.data[4]   # Flex3

        # Pointer
        self.finger_jpos[5] = msg.data[5]   # Adduct
        self.finger_jpos[6] = msg.data[6]   # Flex1
        self.finger_jpos[7] = msg.data[7]   # Flex2
        self.finger_jpos[8] = msg.data[8]   # Flex3

        # Middle
        self.finger_jpos[9] = msg.data[9]     # Adduct
        self.finger_jpos[10] = msg.data[10]   # Flex1
        self.finger_jpos[11] = msg.data[11]   # Flex2
        self.finger_jpos[12] = msg.data[12]   # Flex3
        
        # Ring
        self.finger_jpos[13] = msg.data[13]   # Adduct
        self.finger_jpos[14] = msg.data[14]   # Flex1
        self.finger_jpos[15] = msg.data[15]   # Flex2
        self.finger_jpos[16] = msg.data[16]   # Flex3

        # Pinkie
        self.finger_jpos[17] = msg.data[13]   # Adduct
        self.finger_jpos[18] = msg.data[14]   # Flex1
        self.finger_jpos[19] = msg.data[15]   # Flex2
        self.finger_jpos[20] = msg.data[16]   # Flex3
            
        self.received_sg_cmd_ = True

    def get_sensor_sensordata(self, body_name):
        return self.data.sensordata
    
    def __init__(self):
        path_to_models = '/home/joe/sg_ws/src/mujoco_menagerie/'
        self.deg2rad = math.pi/180
        self.finger_jpos = np.zeros(21)
        self.pos_rh = np.zeros(7)
        self.pos_lh = np.zeros(7)
        self.received_rh_cmd_ = False
        self.received_lh_cmd_ = False
        self.received_sg_cmd_ = False
        
        rospy.init_node('sg_MJC_controller', anonymous=True)
        t0 = rospy.get_time()
        pub = rospy.Publisher('/FingerContacts', Float32MultiArray, queue_size=1)
        sub_rh = rospy.Subscriber('/RightHandPose', TransformStamped, self.callback_rh, queue_size=1)
        sub_lh = rospy.Subscriber('/LeftHandPose', TransformStamped, self.callback_lh, queue_size=1)
        sub_sg = rospy.Subscriber('/Delayed_TPDistance', Float32, self.callback_sg, queue_size=1)
        rate_hz = 200
        r = rospy.Rate(rate_hz)

        contact_msg = Float32MultiArray()
        self.grip_cmd_ = False

        model = mujoco.MjModel.from_xml_path(path_to_models + 'myscenes/allegro_right.xml')
        data = mujoco.MjData(model)

        # create the viewer object
        viewer = mujoco_viewer.MujocoViewer(model, data, hide_menus=False)
        viewer.cam.lookat[0] = 0.3
        viewer.cam.lookat[1] = -0.0
        viewer.cam.lookat[2] = 0.2
        viewer.cam.distance = 1.35
        viewer.cam.elevation = -20
        viewer.cam.azimuth = +0
        viewer._paused = True
        geom_ids_tips = [model.geom('th_tip').id, model.geom('ff_tip').id, model.geom('mf_tip').id, model.geom('rf_tip').id]
        geom_ids_meds = [model.geom('th_medial').id, model.geom('ff_medial').id, model.geom('mf_medial').id, model.geom('rf_medial').id]
        geom_ids_prxs = [model.geom('th_proximal').id, model.geom('ff_proximal').id, model.geom('mf_proximal').id, model.geom('rf_proximal').id]
        # simulate and render
        frm = 0
        while not rospy.is_shutdown():
            if self.received_rh_cmd_:
                # Update position of the wrist base
                #print(self.pos_rh)
                data.qpos[0] = self.pos_rh[0] # x
                data.qpos[1] = self.pos_rh[1] # y
                data.qpos[2] = self.pos_rh[2] # z
                data.qpos[3] = self.pos_rh[3] # ball
                data.qpos[4] = self.pos_rh[4] # ball
                data.qpos[5] = self.pos_rh[5] # ball
                data.qpos[6] = self.pos_rh[6] # ball
                
            if self.received_sg_cmd_:
                # Set joint positions.
                #data.ctrl[3] = self.finger_jpos[5] # pointer base
                data.ctrl[4] = self.finger_jpos[6] # pointer flex 1
                data.ctrl[5] = self.finger_jpos[7] # pointer flex 2
                data.ctrl[6] = self.finger_jpos[8] # pointer flex 3

                #data.ctrl[7] = self.finger_jpos[9]   # middle base
                data.ctrl[8] = self.finger_jpos[10]  # middle flex 1
                data.ctrl[9] = self.finger_jpos[11]  # middle flex 2
                data.ctrl[10] = self.finger_jpos[12] # middle flex 3

                #data.ctrl[11] = self.finger_jpos[13] # ring base
                data.ctrl[12] = self.finger_jpos[14] # ring flex 1
                data.ctrl[13] = self.finger_jpos[15] # ring flex 2
                data.ctrl[14] = self.finger_jpos[16] # ring flex 3
                
                data.ctrl[15] = self.finger_jpos[1]*2   # Thumb    15 = palm fold    
                data.ctrl[16] = self.finger_jpos[0]*3   # Thumb      16 = twist
                data.ctrl[17] = (self.finger_jpos[2]+self.finger_jpos[3]/2)*3   # Thumb      17 = flex 1             
                data.ctrl[18] = (self.finger_jpos[3]/2+self.finger_jpos[4])*1   # Thumb      18 = flex 2

                # TESTING
                #if frm % 100 > 50:
                #    data.ctrl[3] = 10*math.pi/180
                #else:
                #    data.ctrl[3] = 0*math.pi/180

            # Read contacts
            contact_msg.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            for i in range(data.ncon):
                contact = data.contact[i]
                dist = contact.dist         # -ve values are penetration
                friction = contact.friction # 5x1 (tangent1, 2, spin, roll1, 2)
                for g in range(4):
                    if dist < 0:
                        if contact.geom1 == geom_ids_tips[g] or contact.geom2 == geom_ids_tips[g]:
                            if -dist < 0.00001:
                                contact_msg.data[g] += 100   # vib
                                contact_msg.data[g+5] += 100 # force
                        elif contact.geom1 == geom_ids_meds[g] or contact.geom2 == geom_ids_meds[g]:
                            if -dist < 0.00001:
                                contact_msg.data[g] += 80   # vib
                                contact_msg.data[g+5] += 80 # force                            
                        elif contact.geom1 == geom_ids_prxs[g] or contact.geom2 == geom_ids_prxs[g]:
                            if -dist < 0.00001:
                                contact_msg.data[g] += 70   # vib
                                contact_msg.data[g+5] += 70 # force
                        
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
