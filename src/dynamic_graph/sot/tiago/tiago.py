# -*- coding: utf-8 -*-
# Copyright 2016, Olivier STASSE, LAAS-CNRS

from __future__ import print_function

import pinocchio as se3
from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Vector
from dynamic_graph.sot.dynamic_pinocchio import DynamicPinocchio
from dynamic_graph.sot.tiago.robot import AbstractRobot
from pinocchio.robot_wrapper import RobotWrapper

class Tiago(AbstractRobot):
    """
    This class defines a Tiago robot
    """
    defaultFilename = "package://tiago_data/robots/tiago_steel.urdf"
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
        # Gripper position in full configuration: 27:34, and 41:48
        # Small configuration: 36 DOF
        # Full configuration: 50 DOF
        res = config[0:27] + 7 * (0., ) + config[27:34] + 7 * (0., ) + config[34:]
        return res

    def __init__(self, name, initialConfig, device=None, tracer=None, with_wheels=True):
        self.OperationalPointsMap = {
            'wrist': 'arm_7_joint',
            'right-wheel': 'wheel_right_joint',
            'left-wheel': 'wheel_left_joint',
            'mobilebase': 'root_joint',
            'footprint': 'base_footprint_joint',
            'gaze': 'head_2_joint',
        }

        if fromRosParam:
            print("Using ROS parameter \"/robot_description\"")
            rosParamName = "/robot_description"
            import rospy
            if rosParamName not in rospy.get_param_names():
                raise RuntimeError('"' + rosParamName + '" is not a ROS parameter.')
            s = rospy.get_param(rosParamName)

            self.loadModelFromString(s, rootJointType=pinocchio.JointModelFreeFlyer,
                    removeMimicJoints=True)
        else:
            self.loadModelFromUrdf(self.defaultFilename,
                    rootJointType=pinocchio.JointModelFreeFlyer,
                    removeMimicJoints=True)

        # Clean the robot model. Remove:
        # - caster joints
        # - suspension joints
        # - for hey5 hands, remove every hand_* joints except hand_thumb_joint,
        #   hand_index_joint and hand_mrl_joint (mrl = middle, ring, little)
        jointsToRemove = []
        for name in enumerate(self.pinocchioModel.names):
            if not with_wheels and name.startswith("wheel_"):
                jointsToRemove.append(name)
            elif name.startswith('caster'):
                jointsToRemove.append(name)
            elif name.startswith('suspension'):
                jointsToRemove.append(name)
            elif name.startswith('hand_') and \
                    name not in ("hand_thumb_joint", "hand_index_joint", "hand_mrl_joint"):
                jointsToRemove.append(name)

        print("Removing joints " + ", ".join(jointsToRemove))
        self.removeJoints(jointsToRemove)

        assert hasattr(self, "pinocchioModel")
        assert hasattr(self, "pinocchioData")

        self.pinocchioModel = pinocchioRobot.model
        self.pinocchioData = pinocchioRobot.data

        AbstractRobot.__init__(self, name, tracer)

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

        # TODO For position limit, we remove the first value to get
        # a vector of the good size because SoT use euler angles and not
        # quaternions...
        self.device.setPositionBounds(self.pinocchioModel.lowerPositionLimit.T.tolist()[0][1:],
                                      self.pinocchioModel.upperPositionLimit.T.tolist()[0][1:])
        self.device.setVelocityBounds((-self.pinocchioModel.velocityLimit).T.tolist()[0],
                                      self.pinocchioModel.velocityLimit.T.tolist()[0])
        self.device.setTorqueBounds((-self.pinocchioModel.effortLimit).T.tolist()[0],
                                    self.pinocchioModel.effortLimit.T.tolist()[0])

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
            self.dynamic.velocity.value = self.dimension * (0., )

        # Initialize acceleration derivator if chosen
        if self.enableAccelerationDerivator:
            self.accelerationDerivator = \
                Derivator_of_Vector('accelerationDerivator')
            self.accelerationDerivator.dt.value = self.timeStep
            plug(self.velocityDerivator.sout, self.accelerationDerivator.sin)
            plug(self.accelerationDerivator.sout, self.dynamic.acceleration)
        else:
            self.dynamic.acceleration.value = self.dimension * (0., )

        # Create operational points based on operational points map (if provided)
        if self.OperationalPointsMap is not None:
            self.initializeOpPoints()

    def setClosedLoop(self, closedLoop):
        if closedLoop:
            plug(self.device.robotState, self.dynamic.position)
            self.device.setClosedLoop(True)
        else:
            plug(self.device.state, self.dynamic.position)
            self.device.setClosedLoop(False)


__all__ = [Tiago]
