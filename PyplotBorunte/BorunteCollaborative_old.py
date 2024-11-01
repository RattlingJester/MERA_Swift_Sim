"""
@author: Gautam Sinha, Indian Institute of Technology, Kanpur
  (original MATLAB version)
@author: Peter Corke
@author: Samuel Drew
"""

from roboticstoolbox import DHRobot, RevoluteDH
from math import pi
import numpy as np


class BRTIRXZ0805A(DHRobot):
    """
    Class that models a Kuka KR5 manipulator

    ``KR5()`` is a class which models a Kuka KR5 robot and
    describes its kinematic characteristics using standard DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.KR5()
        >>> print(robot)

    Defined joint configurations are:

      - qk1, nominal working position 1
      - qk2, nominal working position 2
      - qk3, nominal working position 3

    .. note::
      - SI units of metres are used.
      - Includes an 11.5cm tool in the z-direction

    :references:
      - https://github.com/4rtur1t0/ARTE/blob/master/robots/KUKA/KR5_arc/parameters.m

    .. codeauthor:: Gautam Sinha, Indian Institute of Technology, Kanpur (original MATLAB version)
    .. codeauthor:: Samuel Drew
    .. codeauthor:: Peter Corke
    """  # noqa

    def __init__(self):
        deg = pi / 180

        L1 = RevoluteDH(
            a = 0, # 0.196
            d = 0.3285, # 0.6215
            alpha = - pi / 2, # pi / 2
            qlim = [-360 * deg, 360 * deg]
        )
        L2 = RevoluteDH(
            a = 0.4895, 
            d = - 0.196, # 0
            alpha = 0, 
            qlim = [-240 * deg, 60 * deg] # -90 degrees is zero for 2nd axis
        )
        L3 = RevoluteDH(
            a = 0,
            d = 0.1708,
            alpha = pi / 2, # pi / 2 # 0
            qlim = [-150 * deg, 150 * deg],
        )
        L4 = RevoluteDH(
            a = 0, # 0.1708
            d = - 0.3285, 
            alpha = pi / 2, 
            qlim = [-360 * deg, 360 * deg],
        )
        L5 = RevoluteDH(
            a = 0, 
            d = 0.1375, # - 0.1375
            alpha = pi / 2, # pi / 2
            qlim = [-150 * deg, 150 * deg],
        )
        L6 = RevoluteDH(
            a = 0, # 0.1375
            d = 0.114,
            alpha = 0, #pi # 0
            qlim = [-360 * deg, 360 * deg])

        L = [L1, L2, L3, L4, L5, L6]

        # Create SerialLink object
        super().__init__(
            L,
            name = "",
            manufacturer = "Borunte",
            meshdir = None,
        )

        self.qr = np.array([180, -90, 90, -90, 0, 0]) * deg # Blueprint position
        self.qz = np.array([pi / 2, - pi / 2, pi / 2, pi / 2, - pi / 2, 0]) # Zero Position

        self.addconfiguration("qz", self.qz)
        self.addconfiguration("qr", self.qr)

if __name__ == "__main__":  # pragma nocover
    robot = BRTIRXZ0805A()
    print(robot)
