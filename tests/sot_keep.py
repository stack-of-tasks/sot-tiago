from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core import Task, FeatureGeneric, GainAdaptive, FeaturePosture
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph import plug

n = "posture"
tp = Task ('task' + n)
tp.dyn = robot.dynamic
tp.feature = FeaturePosture('feature_'+n)

q = list(robot.device.robotState.value)
tp.feature.state.value = q
tp.feature.posture.value = q

robotDim = robot.dynamic.getDimension()
for i in range(8, robotDim):
    tp.feature.selectDof (i, True)

tp.gain = GainAdaptive("gain_"+n)
tp.add(tp.feature.name)

# Connects the dynamics to the current feature of the posture task
plug(robot.dynamic.position, tp.feature.state)

# Set the gain of the posture task
setGain(tp.gain,(4.9,0.9,0.01,0.9))
plug(tp.gain.gain, tp.controlGain)
plug(tp.error, tp.gain.error)

def getJointIdxQ (name):
    model = robot.pinocchioModel
    jid = model.getJointId (name)
    return model.joints[jid].idx_q - 1

# Set initial state.
#q[getJointIdxQ("torso_lift_joint")] = 0.25
#q[getJointIdxQ("arm_1_joint")] = 0.2
#q[getJointIdxQ("arm_2_joint")] = 0.35
#q[getJointIdxQ("arm_3_joint")] = -0.2
#q[getJointIdxQ("arm_4_joint")] = 1.94
#q[getJointIdxQ("arm_5_joint")] = -1.57
#q[getJointIdxQ("arm_6_joint")] = 1.37
#q[getJointIdxQ("arm_7_joint")] = -1.39
#q[getJointIdxQ("gripper_finger_joint")] =  0.002000
#q[getJointIdxQ("head_1_joint")] = 0.
#q[getJointIdxQ("head_2_joint")] = 0.
#robot.device.state.value = q

# Set reference posture.
#q[getJointIdxQ("torso_lift_joint")] = 0.204707
#q[getJointIdxQ("arm_1_joint")] = 0.200167
#q[getJointIdxQ("arm_2_joint")] = -1.500333
#q[getJointIdxQ("arm_3_joint")] = -0.283931
#q[getJointIdxQ("arm_4_joint")] = 1.366962
#q[getJointIdxQ("arm_5_joint")] = -1.565381
#q[getJointIdxQ("arm_6_joint")] = 1.347353
#q[getJointIdxQ("arm_7_joint")] = 0.015935
#q[getJointIdxQ("gripper_finger_joint")] =  0.002000
#q[getJointIdxQ("head_1_joint")] = -0.5
##q[getJointIdxQ("head_1_joint")] = -0.004879
#q[getJointIdxQ("head_2_joint")] = -0.016528

#q[getJointIdxQ("torso_lift_joint")] = 0.25
#q[getJointIdxQ("arm_1_joint")] = 0.2
#q[getJointIdxQ("arm_2_joint")] = 0.35
#q[getJointIdxQ("arm_3_joint")] = -0.2
#q[getJointIdxQ("arm_4_joint")] = 1.94
#q[getJointIdxQ("arm_5_joint")] = -1.57
#q[getJointIdxQ("arm_6_joint")] = 1.37
#q[getJointIdxQ("arm_7_joint")] = -1.39
#q[getJointIdxQ("gripper_finger_joint")] =  0.002000
#q[getJointIdxQ("head_1_joint")] = 0.05
##q[getJointIdxQ("head_1_joint")] = 0.
#q[getJointIdxQ("head_2_joint")] = 0.
#tp.feature.posture.value = q

from dynamic_graph.sot.tiago.diff_drive_controller import HolonomicProjection
projection = HolonomicProjection("projection")
projection.setSize (robot.dynamic.getDimension())
projection.setLeftWheel (6)
projection.setRightWheel (7)
# The wheel separation could be obtained with pinocchio.
# See pmb2_description/urdf/base.urdf.xacro
projection.setWheelRadius (0.0985)
projection.setWheelSeparation (0.4044)
plug (robot.dynamic.mobilebase, projection.basePose)

from dynamic_graph.sot.core import SOT
sot = SOT('sot')
sot.setSize(robot.dynamic.getDimension())

plug(projection.projection, sot.proj0)

plug(sot.control,robot.device.control)

#from dynamic_graph.ros import RosPublish
#ros_publish_state = RosPublish ("ros_publish_state")
#ros_publish_state.add ("vector", "state", "/sot_control/state")
#plug (robot.device.state, ros_publish_state.state)
#robot.device.after.addDownsampledSignal ("ros_publish_state.trigger", 100)

robot.device.control.recompute(0)
