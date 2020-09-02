# -*- coding: utf-8 -*-
# Copyright 2016, Olivier STASSE, LAAS-CNRS

from __future__ import print_function

import pinocchio as se3
from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Vector
from dynamic_graph.sot.dynamic_pinocchio.dynamic import DynamicPinocchio
from dynamic_graph.sot.dynamic_pinocchio.humanoid_robot import AbstractRobot
import pinocchio

class Tiago(AbstractRobot):
    """
    This class defines a Tiago robot
    """

    defaultFilename = "package://tiago_data/robots/tiago_steel.urdf"

    def defineHalfSitting(self, q):
        """
        q is the configuration to fill.

        When this function is called, the attribute pinocchioModel has been filled.
        """
        model = self.pinocchioModel
        # set arm position
        self.setJointValueInConfig(q,
                [ "arm_{}_joint".format(i+1) for i in range(7) ],
                (0., -1.569796, -1.569796, 2.355194, 0., 0., 0.,))

    def __init__(self, robotName, device=None, tracer=None, with_wheels=True, fromRosParam=False):
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
        for name in self.pinocchioModel.names:
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

        AbstractRobot.__init__(self, robotName, tracer)

        # Create rigid body dynamics model and data (pinocchio)
        self.dynamic = DynamicPinocchio(self.name + "_dynamic")
        self.dynamic.setModel(self.pinocchioModel)
        self.dynamic.setData(self.pinocchioData)
        self.dynamic.displayModel()
        self.dimension = self.dynamic.getDimension()

        self.device = device
        self.initializeRobot()

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

    def _initialize(self):
        AbstractRobot._initialize(self)
        self.OperationalPoints.extend(['wrist', 'left-wheel', 'right-wheel', 'footprint', 'mobilebase', 'gaze'])


__all__ = [Tiago]
