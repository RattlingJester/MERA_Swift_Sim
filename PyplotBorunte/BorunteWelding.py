"""
@author: Gautam Sinha, Indian Institute of Technology, Kanpur
  (original MATLAB version)
@author: Peter Corke
@author: Samuel Drew
"""

from roboticstoolbox import DHRobot, RevoluteDH
from math import pi
import numpy as np


class BRTIRUS2520B(DHRobot):
    def __init__(self):
        deg = pi / 180

        L1 = RevoluteDH(
            a = 0.401154, # 0.401154
            d = 0.6215, # 0.6215
            alpha = - pi / 2, 
            qlim = [-160 * deg, 160 * deg]
        )
        L2 = RevoluteDH(
            a = 0.799142, # 0.799142
            d = 0, 
            alpha = 0, 
            qlim = [-125 * deg, -5 * deg] # -90 degrees is zero for 2nd axis -5 : -120
        )
        L3 = RevoluteDH(
            a = 0.318414, # 0.318414
            d = - 0.038871, # - 0.038871
            alpha = - pi / 2,
            qlim = [-75 * deg, 100 * deg]
        )
        L4 = RevoluteDH(
            a = 0,
            d = 1.143634, # - 1.143634
            alpha = - pi / 2,
            qlim = [-180 * deg, 180 * deg]
        )
        L5 = RevoluteDH(
            a = 0.0, 
            d = 0,
            alpha = pi / 2, 
            qlim = [-95 * deg, 95 * deg]
        )
        L6 = RevoluteDH(
            a = 0, 
            d = 0.277, # 0.277
            alpha = pi, # pi
            qlim = [-360 * deg, 360 * deg])

        L = [L1, L2, L3, L4, L5, L6]

        # Create SerialLink object
        super().__init__(
            L,
            name = "BRTIRUS2520B",
            manufacturer = "Borunte",
            meshdir = None,
        )

        self.qz = np.array([0, -90, 0, 0, 0, 0]) * deg # Zero Position
        self.qh = np.array([0, 0, -90, 0, 0, 0]) * deg # Horizontal Position

        self.addconfiguration("qz", self.qz)
        self.addconfiguration("qh", self.qh)

        self.addconfiguration_attr(
            "qk1", np.array([pi / 4, pi / 3, pi / 4, pi / 6, pi / 4, pi / 6])
        )
        self.addconfiguration_attr(
            "qk2", np.array([pi / 4, pi / 3, pi / 6, pi / 3, pi / 4, pi / 6])
        )
        self.addconfiguration_attr(
            "qk3", np.array([pi / 6, pi / 3, pi / 6, pi / 3, pi / 6, pi / 3])
        )

if __name__ == "__main__":
    robot = BRTIRUS2520B()
    print(robot)
