#!/usr/bin/env python
import rospy
from controller_manager_msgs.srv import SwitchController

rospy.init_node('StoppingControllers')
swCtl = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
resp = swCtl(stop_controllers=[
    'mobile_base_controller', 'arm_controller', 'head_controller', 'torso_controller', 'gripper_controller',  
   # 'diff_drive_controller', 'effort_controllers', 'force_torque_sensor_controller',
   # 'imu_sensor_controller',
   # 'pos_vel_acc_controllers',
   # 'pos_vel_controllers',
   # 'position_controllers',
   # 'velocity_controllers',
   # 'temperature_sensor_controller'
	
],
             strictness=2)
