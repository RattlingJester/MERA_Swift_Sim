import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

import BorunteWelding

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

robot = BorunteWelding.BRTIRUS2520B()

# q = robot.toradians([
#                     28.557, 
#                     map_range(-9.432,  -85, 35, -5, -125), # -90 degrees is zero for 2nd axis
#                     map_range(-66.590, -100, 75, 100, -75), 
#                     179.999, 
#                     13.979, 
#                     -61.442
#                     ])

Tep1 = SE3.Trans(2, 0, 1) * SE3.OA([0, 1, 0], [-1, 0, 0]) # XYZ Coorinates and End-effector orientation. (end-effector z-axis down (A=-Z) and finger orientation parallel to y-axis (O=+Y)
Tep2 = SE3.Trans(2, 0, 1.5) * SE3.OA([0, 1, 0], [-1, 0, 0])
sol1 = robot.ik_LM(Tep1) # solve IK
sol2 = robot.ik_LM(Tep2)
q_pickup = sol1[0]
q_Middle = sol2[0]

# Printing angles:
print(np.around(map_range(robot.todegrees(q_pickup)[0], -160, 160, -160, 160), decimals=3)) # Axis 1
print(np.around(map_range(robot.todegrees(q_pickup)[1], -85, 35, -5, -125), decimals=3))    # Axis 2
print(np.around(map_range(robot.todegrees(q_pickup)[2], -100, 75, 100, -75), decimals=3))   # Axis 3
print(np.around(map_range(robot.todegrees(q_pickup)[3], -180, 180, -180, 180), decimals=3)) # Axis 4
print(np.around(map_range(robot.todegrees(q_pickup)[4], -95, 95, -95, 95), decimals=3))     # Axis 5
print(np.around(map_range(robot.todegrees(q_pickup)[5], -360, 360, -360, 360), decimals=3)) # Axis 6

print(robot.fkine(robot.qz))

# qt = rtb.jtraj(robot.qz, q_pickup, 200)

viapoints = np.ndarray(shape = (3,6), dtype = float, buffer = np.array([robot.qz, q_Middle, q_pickup]))
dt = 0.1
tacc = 0.5
qdmax = 1
qm = rtb.mstraj(viapoints, dt, tacc, qdmax)
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

#robot.plot(robot.qz, block=True)
env = robot.plot(qm.q, backend='pyplot')
env.hold()