import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

import BorunteCollaborative

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

robot = BorunteCollaborative.BRTIRXZ0805A()

# q = robot.toradians([
#                     28.557, 
#                     map_range(-9.432,  -85, 35, -5, -125), # -90 degrees is zero for 2nd axis
#                     map_range(-66.590, -100, 75, 100, -75), 
#                     179.999, 
#                     13.979, 
#                     -61.442
#                     ])

Tep = SE3.Trans(0.5, 0, 0.5) * SE3.OA([0, 1, 0], [-1, 0, 0]) # XYZ Coorinates and End-effector orientation
sol = robot.ik_LM(Tep) # solve IK
q_pickup = sol[0]

# Printting angles:
print(np.around(map_range(robot.todegrees(q_pickup)[0], -360, 360, -360, 360), decimals=3)) # Axis 1
print(np.around(map_range(robot.todegrees(q_pickup)[1], -240, 60, 150, -150), decimals=3))    # Axis 2
print(np.around(map_range(robot.todegrees(q_pickup)[2], -150, 150, 150, -150), decimals=3))   # Axis 3
print(np.around(map_range(robot.todegrees(q_pickup)[3], -360, 360, -360, 360), decimals=3)) # Axis 4
print(np.around(map_range(robot.todegrees(q_pickup)[4], -150, 150, 150, -150), decimals=3))     # Axis 5
print(np.around(map_range(robot.todegrees(q_pickup)[5], -360, 360, -360, 360), decimals=3)) # Axis 6

print(robot.fkine(q_pickup))

qt = rtb.jtraj(robot.qr, q_pickup, 50)

# viapoints = np.ndarray(shape = (4,6), dtype = float, buffer = np.array([robot.qz, sol[0], robot.qh, robot1.qz]))
# dt = 0.05
# tacc = 0.5
# qdmax = 1
# qm = rtb.mstraj(viapoints, dt, tacc, qdmax)
# print(robot.todegrees(qm.q[0]))
# print(robot.todegrees(robot.qz))

# File output
# f = open("points.txt", "w+")

# f.write(str(np.around(map_range(robot.todegrees(q_pickup)[0], -160, 160, -160, 160), decimals=3)) + '\n') # Axis 1
# f.write(str(np.around(map_range(robot.todegrees(q_pickup)[1], -85, 35, -5, -125), decimals=3)) + '\n')    # Axis 2
# f.write(str(np.around(map_range(robot.todegrees(q_pickup)[2], -100, 75, 100, -75), decimals=3)) + '\n')   # Axis 3
# f.write(str(np.around(map_range(robot.todegrees(q_pickup)[3], -180, 180, -180, 180), decimals=3)) + '\n') # Axis 4
# f.write(str(np.around(map_range(robot.todegrees(q_pickup)[4], -95, 95, -95, 95), decimals=3)) + ' \n')     # Axis 5
# f.write(str(np.around(map_range(robot.todegrees(q_pickup)[5], -360, 360, -360, 360), decimals=3))) # Axis 6

env = robot.plot(qt.q, backend='pyplot')
env.hold()