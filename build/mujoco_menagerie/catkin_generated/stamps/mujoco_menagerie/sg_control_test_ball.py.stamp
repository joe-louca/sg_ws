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
    def callback_pol(self, msg):       
        # Thumb
        self.pos_pol[0] = msg.transform.translation.x()   # x
        self.pos_pol[1] = msg.transform.translation.y()
        self.pos_pol[2] = msg.transform.translation.z()
        self.received_pol_cmd_ = True

    
    def __init__(self):
        path_to_models = '/home/joe/sg_ws/src/mujoco_menagerie/'
        self.deg2rad = math.pi/180
        self.finger_jpos = np.zeros(21)
        self.received_pol_cmd_ = False
        
        rospy.init_node('sg_MJC_controller', anonymous=True)
        t0 = rospy.get_time()
        sub = rospy.Subscriber('/WristPose', TransformStamped, self.callback_pol, queue_size=1)
        rate_hz = 200
        r = rospy.Rate(rate_hz)


        model = mujoco.MjModel.from_xml_path(path_to_models + 'test_ball/scene_test_ball.xml')
        data = mujoco.MjData(model)

        # create the viewer object
        viewer = mujoco_viewer.MujocoViewer(model, data, hide_menus=False)
        viewer.cam.lookat[0] = 0.3
        viewer.cam.lookat[1] = -0.0
        viewer.cam.lookat[2] = 0.2
        viewer.cam.distance = 1.35
        viewer.cam.elevation = -20
        viewer.cam.azimuth = +20
        viewer._paused = True
        geom_ids_tips = [model.geom('TH_TIP').id, model.geom('F1_TIP').id, model.geom('F2_TIP').id, model.geom('F3_TIP').id, model.geom('F4_TIP').id]
        geom_ids_meds = [model.geom('TH_MED').id, model.geom('F1_MED').id, model.geom('F2_MED').id, model.geom('F3_MED').id, model.geom('F4_MED').id]
        geom_ids_prxs = [model.geom('TH_PRX').id, model.geom('F1_PRX').id, model.geom('F2_PRX').id, model.geom('F3_PRX').id, model.geom('F4_PRX').id]
        
        # simulate and render
        frm = 0
        while not rospy.is_shutdown():
            if self.received_sg_cmd_:
                # Set positions
                data.ctrl[0] = self.pos_pol[0]   # Thumb      twist    lh_A_THJ5
                data.ctrl[1] = self.pos_pol[1]   # Thumb      twist    lh_A_THJ5
                data.ctrl[2] = self.pos_pol[2]   # Thumb      twist    lh_A_THJ5

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
