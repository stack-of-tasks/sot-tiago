from math import cos, pi, sin

import matplotlib.pyplot as plt

from dynamic_graph.sot.tiago.diff_drive_controller import DiffDriveController

ddc = DiffDriveController("controller")

ddc.setWheelSeparation(0.4)
ddc.setWheelRadius(0.2)
ddc.setPeriod(0.1)

ddc.setOpenLoop(True)

# Initialize
ddc.baseVelIn.value = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
ddc.wheelsVelOut.recompute(0)

# ddc.baseVelIn.value = ( 1., 0., 0., 0., 0., 0. )
# ddc.baseVelIn.value = ( 0., 0., 0., 0., 0., 1. )


def run(N):
    t0 = ddc.basePoseOut.time
    for t in range(t0, t0 + N):
        ddc.baseVelIn.time = t
        ddc.wheelsVelOut.recompute(t)
        ddc.basePoseOut.recompute(t)
        # ddc.baseVelOut  .recompute(t)
        # if t == 1: print ddc.wheelsVelOut.value
        print(ddc.basePoseOut.value)
        # print(ddc.baseVelOut.value)


def runCircle(N):
    XYT = []
    t0 = ddc.basePoseOut.time + 1
    for t in range(t0, t0 + N + 1):
        x, y, theta = ddc.basePoseOut.value
        ddc.baseVelIn.value = (
            cos(theta),
            sin(theta),
            0.0,
            0.0,
            0.0,
            2 * pi / (N * ddc.getPeriod()),
        )
        ddc.baseVelIn.time = t

        ddc.wheelsVelOut.recompute(t)
        ddc.basePoseOut.recompute(t)

        XYT.append(ddc.basePoseOut.value)
    return XYT


def runAndPlotCircle(N):

    xyt = runCircle(N)
    plt.plot([x[0] for x in xyt], [x[1] for x in xyt])
    plt.show()
