import mujoco
import mujoco_viewer
import time
import math
import rospy
from std_msgs.msg import Float32

class SG_CONTROL:
    def callback(self, msg):
        self.thumb2pointer_distance = msg.data
        self.grip_cmd_ = True
        
    def __init__(self):
        path_to_models = '/home/joe/sg_ws/src/mujoco_menagerie/'
        
        rospy.init_node('sg_MJC_controller', anonymous=True)
        #pub = rospy.Publisher('/contacts', Float32, queue_size=1)
        sub = rospy.Subscriber('/TP_distance', Float32, self.callback, queue_size=1)
        rate_hz = 100
        r = rospy.Rate(rate_hz)
        
        self.grip_cmd_ = False

        sg_max_distance = 10

        #model = mujoco.MjModel.from_xml_path('shadow_hand/scene_left.xml')
        model = mujoco.MjModel.from_xml_path(path_to_models + 'robotiq_2f85/scene.xml')
        data = mujoco.MjData(model)

        # create the viewer object
        viewer = mujoco_viewer.MujocoViewer(model, data, hide_menus=False)

        # simulate and render
        #while not rospy.is_shutdown():
        for frm in range(50000):
            if self.grip_cmd_:
                grip_pos = ((sg_max_distance - self.thumb2pointer_distance) / sg_max_distance) * 255

                # Set grip position: 0=open, 255=closed
                #data.ctrl[0] = math.cos(frm / 100) * 255
                data.ctrl[0] = grip_pos

            # Read contacts 
            #pub.publish(contact_msg)
            
            if viewer.is_alive:
                mujoco.mj_step(model, data)
                viewer.render()
            else:
                break
            time.sleep(0.001)
            #r.sleep

        # close
        viewer.close()

if __name__ == '__main__':
    foo = SG_CONTROL()
