# -*- coding: utf-8 -*-
# Copyright 2018, Joseph Mirabel,

# sys.argv is not defined when running the remove interpreter, but it is
# required by rospy
import sys

from dynamic_graph.entity import PyEntityFactoryClass
from dynamic_graph.sot.tiago.steel.robot import TiagoSteel as Robot

if not hasattr(sys, 'argv'):
    sys.argv = [
        "dynamic_graph",
    ]

print("Prologue TIAGO Steel Robot")


# Create the device.
# This entity behaves exactly like robotsimu except:
# 1. it does not provide the increment command
# 2. it forwards the robot control to the sot-abstract
#    controller.
def makeRobot(with_wheels=True):
    DeviceTiago = PyEntityFactoryClass('DeviceTiago')

    # Create the robot using the device.
    robot = Robot(name='tiago', device=DeviceTiago('TIAGOSTEEL'), with_wheels=with_wheels, fromRosParam=True)

    return robot


####################################
#        --- IMPORTANT ---         #
#                                  #
# THIS FILE MUST NEVER BE CHANGED. #
# TO RUN YOUR EXPERIMENT, PLEASE   #
# WRITE A SEPARATE PYTHON MODULE   #
# AND LAUNCH IT USING dg-remote!   #
####################################
