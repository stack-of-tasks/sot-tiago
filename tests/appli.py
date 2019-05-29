from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from numpy import eye
from dynamic_graph import plug

taskRH    = MetaTaskKine6d('rh',robot.dynamic,'rh',robot.OperationalPointsMap['wrist'])
handMgrip = eye(4); handMgrip[0:3,3] = (0.1,0,0)
taskRH.opmodif = matrixToTuple(handMgrip)
taskRH.feature.frame('desired')

from dynamic_graph.sot.tiago.diff_drive_controller import HolonomicProjection
projection = HolonomicProjection("projection")
projection.setSize (robot.dynamic.getDimension())
plug (robot.dynamic.mobilebase, projection.basePose)

from dynamic_graph.sot.core import SOT
sot = SOT('sot')
sot.setSize(robot.dynamic.getDimension())

plug(projection.projection, sot.proj0)

plug(sot.control,robot.device.control)

from dynamic_graph.ros import RosPublish
ros_publish_state = RosPublish ("ros_publish_state")
ros_publish_state.add ("vector", "state", "/sot_control/state")
from dynamic_graph import plug
plug (robot.device.state, ros_publish_state.state)
robot.device.after.addDownsampledSignal ("ros_publish_state.trigger", 100)

robot.device.control.recompute(0)
