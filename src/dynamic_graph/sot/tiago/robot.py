# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST

from __future__ import print_function

from functools import reduce

from dynamic_graph import plug
from dynamic_graph.sot.core import OpPointModifier, RobotSimu
from dynamic_graph.sot.core.derivator import Derivator_of_Vector
from dynamic_graph.sot.dynamic_pinocchio import DynamicPinocchio, AbstractRobot
from dynamic_graph.tools import addTrace
from dynamic_graph.tracer_real_time import TracerRealTime

class Robot(AbstractRobot):
    def __init__(self, name, pinocchio_model, pinocchio_data, initialConfig, OperationalPointsMap=None, tracer=None):
        AbstractRobot.__init__(self, name, tracer)

        self.OperationalPointsMap = OperationalPointsMap

        self.dynamic = DynamicPinocchio(self.name + "_dynamic")
        self.dynamic.setModel(pinocchio_model)
        self.dynamic.setData(pinocchio_data)
        self.dimension = self.dynamic.getDimension()

        self.device = RobotSimu(self.name + "_device")

        self.device.resize(self.dynamic.getDimension())
        self.halfSitting = initialConfig
        self.device.set(self.halfSitting)
        plug(self.device.state, self.dynamic.position)

        # TODO For position limit, we remove the first value to get
        # a vector of the good size because SoT use euler angles and not
        # quaternions...
        self.device.setPositionBounds(pinocchio_model.lowerPositionLimit.tolist()[1:],
                                      pinocchio_model.upperPositionLimit.tolist()[1:])
        self.device.setVelocityBounds((-pinocchio_model.velocityLimit).tolist(),
                                      pinocchio_model.velocityLimit.tolist())
        self.device.setTorqueBounds((-pinocchio_model.effortLimit).tolist(),
                                    pinocchio_model.effortLimit.tolist())

        if self.enableVelocityDerivator:
            self.velocityDerivator = Derivator_of_Vector('velocityDerivator')
            self.velocityDerivator.dt.value = self.timeStep
            plug(self.device.state, self.velocityDerivator.sin)
            plug(self.velocityDerivator.sout, self.dynamic.velocity)
        else:
            self.dynamic.velocity.value = self.dimension * (0., )

        if self.enableAccelerationDerivator:
            self.accelerationDerivator = \
                Derivator_of_Vector('accelerationDerivator')
            self.accelerationDerivator.dt.value = self.timeStep
            plug(self.velocityDerivator.sout, self.accelerationDerivator.sin)
            plug(self.accelerationDerivator.sout, self.dynamic.acceleration)
        else:
            self.dynamic.acceleration.value = self.dimension * (0., )
        if self.OperationalPointsMap is not None:
            self.initializeOpPoints()

    def _initialize(self):
        AbstractRobot._initialize(self)
        self.OperationalPoints.extend(['wrist', 'left-wheel', 'right-wheel', 'footprint', 'mobilebase', 'gaze'])
