# -*- coding: utf-8 -*-
# Copyright 2018, Joseph Mirabel, 
#
# This file is part of sot_tiago.
# sot_tiago is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot_tiago is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot_tiago. If not, see <http://www.gnu.org/licenses/>.
print("Prologue TIAGO Steel Robot")

from dynamic_graph.entity import PyEntityFactoryClass
from dynamic_graph.sot.tiago.steel.robot import Robot

# Create the device.
# This entity behaves exactly like robotsimu except:
# 1. it does not provide the increment command
# 2. it forwards the robot control to the sot-abstract
#    controller.
DeviceTiago = PyEntityFactoryClass('DeviceTiago')

# Create the robot using the device.
robot = Robot(name = 'robot', device = DeviceTiago('TIAGOSTEEL'))

robot.dynamic.com.recompute (0)
_com = robot.dynamic.com.value
robot.device.zmp.value = (_com[0], _com[1], 0.)

__all__ = ["robot"]

####################################
#        --- IMPORTANT ---         #
#                                  #
# THIS FILE MUST NEVER BE CHANGED. #
# TO RUN YOUR EXPERIMENT, PLEASE   #
# WRITE A SEPARATE PYTHON MODULE   #
# AND LAUNCH IT USING dg-remote!   #
####################################
