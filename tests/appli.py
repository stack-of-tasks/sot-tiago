# flake8: noqa
from dynamic_graph import plug
from dynamic_graph.ros import RosPublish
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.tiago.diff_drive_controller import HolonomicProjection
from numpy import eye

taskRH = MetaTaskKine6d('rh', robot.dynamic, 'rh', robot.OperationalPointsMap['wrist'])
handMgrip = eye(4)
handMgrip[0:3, 3] = (0.1, 0, 0)
taskRH.opmodif = matrixToTuple(handMgrip)
taskRH.feature.frame('desired')

projection = HolonomicProjection("projection")
projection.setSize(robot.dynamic.getDimension())
projection.setLeftWheel(6)
projection.setRightWheel(7)
# The wheel separation could be obtained with pinocchio.
# See pmb2_description/urdf/base.urdf.xacro
projection.setWheelRadius(0.0985)
projection.setWheelSeparation(0.4044)
plug(robot.dynamic.mobilebase, projection.basePose)

sot = SOT('sot')
sot.setSize(robot.dynamic.getDimension())

plug(projection.projection, sot.proj0)

plug(sot.control, robot.device.control)

ros_publish_state = RosPublish("ros_publish_state")
ros_publish_state.add("vector", "state", "/sot_control/state")
plug(robot.device.state, ros_publish_state.state)
robot.device.after.addDownsampledSignal("ros_publish_state.trigger", 100)

robot.device.control.recompute(0)
