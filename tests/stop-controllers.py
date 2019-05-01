#!/usr/bin/env python
import rospy

from controller_manager_msgs.srv import SwitchController

rospy.init_node('StoppingControllers')
swCtl=rospy.ServiceProxy('/controller_manager/switch_controller',SwitchController)
resp = swCtl(stop_controllers=['arm_controller','head_controller','torso_controller','gripper_controller'],strictness=2)

