# -*- coding: utf-8 -*-
# Copyright 2016, Olivier STASSE, LAAS-CNRS
#
# This file is part of TIAGOController.
# TIAGOController is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# TIAGOController is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# TIAGOController. If not, see <http://www.gnu.org/licenses/>.

from __future__ import print_function

import numpy as np

from dynamic_graph.sot.tiago.robot import AbstractRobot
from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio
from dynamic_graph import plug

import pinocchio as se3
from rospkg import RosPack

# Internal helper tool.
def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)

class Tiago(AbstractRobot):
    """
    This class defines a Tiago robot
    """

    """
    TODO: Confirm the position and existence of these sensors
    accelerometerPosition = np.matrix ((
            (1., 0., 0., -.13,),
            (0., 1., 0., 0.,),
            (0., 0., 1., .118,),
            (0., 0., 0., 1.,),
            ))

    gyrometerPosition = np.matrix ((
            (1., 0., 0., -.13,),
            (0., 1., 0., 0.,),
            (0., 0., 1., .118,),
            (0., 0., 0., 1.,),
            ))
    """
    def smallToFull(self, config):
        #Gripper position in full configuration: 27:34, and 41:48
        #Small configuration: 36 DOF
        #Full configuration: 50 DOF
        res = config[0:27] + 7*(0.,) + config[27:34]+ 7*(0.,)+config[34:]
        return res

    def __init__(self, name, initialConfig, device = None, tracer = None):
        self.OperationalPointsMap = {'wrist'       : 'arm_7_joint',
                                     'right-wheel' : 'wheel_right_joint',
                                     'left-wheel'  : 'wheel_left_joint',
                                     'mobilebase'  : 'root_joint',
                                     'footprint'   : 'base_footprint_joint',
                                     'gaze'        : 'head_2_joint',
                                     }

        from rospkg import RosPack
        rospack = RosPack()
        self.urdfFile = rospack.get_path('tiago_data')+"/robots/tiago_steel.urdf"
        self.urdfDir = [rospack.get_path('tiago_data')+"/../"]

        # Create a wrapper to access the dynamic model provided through an urdf file.
        from pinocchio.robot_wrapper import RobotWrapper
        import pinocchio as se3
        from dynamic_graph.sot.dynamics_pinocchio import fromSotToPinocchio
        pinocchioRobot = RobotWrapper()
        pinocchioRobot.initFromURDF(self.urdfFile,
                                    self.urdfDir,
                                    se3.JointModelFreeFlyer())

        self.pinocchioModel = pinocchioRobot.model
        self.pinocchioData = pinocchioRobot.data

        AbstractRobot.__init__ (self, name, tracer)

        # Create rigid body dynamics model and data (pinocchio)
        self.dynamic = DynamicPinocchio(self.name + "_dynamic")
        self.dynamic.setModel(self.pinocchioModel)
        self.dynamic.setData(self.pinocchioData)
        self.dimension = self.dynamic.getDimension()

        # Initialize device
        self.device = device
        self.timeStep = self.device.getTimeStep()
        self.device.resize(self.dynamic.getDimension())
        self.halfSitting = initialConfig
        self.device.set(self.halfSitting)
        plug(self.device.state, self.dynamic.position)

        self.dimension = self.dynamic.getDimension()
        self.plugVelocityFromDevice = True
        self.dynamic.displayModel()

        # Initialize velocity derivator if chosen
        if self.enableVelocityDerivator:
            self.velocityDerivator = Derivator_of_Vector('velocityDerivator')
            self.velocityDerivator.dt.value = self.timeStep
            plug(self.device.state, self.velocityDerivator.sin)
            plug(self.velocityDerivator.sout, self.dynamic.velocity)
        else:
            self.dynamic.velocity.value = self.dimension*(0.,)

        # Initialize acceleration derivator if chosen
        if self.enableAccelerationDerivator:
            self.accelerationDerivator = \
                Derivator_of_Vector('accelerationDerivator')
            self.accelerationDerivator.dt.value = self.timeStep
            plug(self.velocityDerivator.sout,
                 self.accelerationDerivator.sin)
            plug(self.accelerationDerivator.sout, self.dynamic.acceleration)
        else:
            self.dynamic.acceleration.value = self.dimension*(0.,)

        # Create operational points based on operational points map (if provided)
        if self.OperationalPointsMap is not None:
            self.initializeOpPoints()

    def setClosedLoop(self, closedLoop):
        if closedLoop:
            plug(self.device.robotState, self.dynamic.position)
            self.device.setClosedLoop (True)
        else:
            plug(self.device.state, self.dynamic.position)
            self.device.setClosedLoop (False)

__all__ = [Tiago]
