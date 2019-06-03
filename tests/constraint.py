# We want SOT to compute a velocity of the form:
#
#   v = (v_ff, v_robot)
#
# where v_ff, the base velocity wrt the world frame, is such that:
#   v_ff = oMb.act( (l, 0, 0, 0, 0, w) ) = K * (v, w)
# where oMb is the base pose wrt to the world frame.
# l and w are the linear and angular velocity of the base, in the base frame.

from dynamic_graph.sot.tiago.diff_drive_controller import HolonomicProjection
projection = HolonomicProjection("projection")
projection.setSize (10)
projection.setWheelSeparation (0.25)
projection.setWheelRadius     (0.2)

from dynamic_graph.sot.core import PoseRollPitchYawToMatrixHomo
matrixHomo = PoseRollPitchYawToMatrixHomo('convert')

from dynamic_graph import plug
plug (matrixHomo.sout, projection.basePose)

def compute (pose):
    t = matrixHomo.sin.time
    matrixHomo.sin.time = t + 1
    matrixHomo.sin.value = pose
    projection.projection.recompute (t+1)
    return projection.projection.value

def setWheels ():
    projection.setLeftWheel  (7)
    projection.setRightWheel (8)

zero = (0, 0, 0, 0, 0, 0)


# We should use Constraint but I cannot understand the way the are used.
# The code of class dynamicgraph::sot::Sot is very obscure on this point.

# Instead, we use a task with the following properties:
# - dimension: 4
# - the error is always zero.
# - the Jacobian J (dim 4*N)  is such that
#   * v = K * vv, where K (dim N*N-4) is a base of the null space of J.

# To compute J:
# 1. compute K
# 2. compute I (dim N * 4) vectors orthogonal to all cols of K (i.e. I^T * K = 0)
# 3. Left J be I^T
# Find J such that J * K = 0 ?
