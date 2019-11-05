# -*- coding: utf-8 -*-
# Copyright 2018, Joseph Mirabel,
#
# This file is part of sot-tiago.
# sot-tiago is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-tiago is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-tiago. If not, see <http://www.gnu.org/licenses/>.

from dynamic_graph.entity import PyEntityFactoryClass
from dynamic_graph.sot.tiago.steel.robot import TiagoSteel as Robot

print("Prologue TIAGO Steel Robot")


# Create the device.
# This entity behaves exactly like robotsimu except:
# 1. it does not provide the increment command
# 2. it forwards the robot control to the sot-abstract
#    controller.
def makeRobot(with_wheels=True):
    DeviceTiago = PyEntityFactoryClass('DeviceTiago')

    # Create the robot using the device.
    robot = Robot(name='robot', device=DeviceTiago('TIAGOSTEEL'), with_wheels=with_wheels)

    return robot


####################################
#        --- IMPORTANT ---         #
#                                  #
# THIS FILE MUST NEVER BE CHANGED. #
# TO RUN YOUR EXPERIMENT, PLEASE   #
# WRITE A SEPARATE PYTHON MODULE   #
# AND LAUNCH IT USING dg-remote!   #
####################################
