#!/usr/bin/env python

import numpy as np
from roboticstoolbox.robot.Robot import Robot
from math import pi


class X_5002(Robot):
    """
    Class that imports a Puma 560 URDF model

    ``Puma560()`` is a class which imports a Unimation Puma560 robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.Puma560()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. warning:: This file has been modified so that the zero-angle pose is the
        same as the DH model in the toolbox. ``j3`` rotation is changed from
        -𝜋/2 to 𝜋/2.  Dimensions are also slightly different.  Both models
        include the pedestal height.

    .. note:: The original file is from https://github.com/nimasarli/puma560_description/blob/master/urdf/puma560_robot.urdf.xacro

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "MERA5002_description/urdf/MERA5002_robot.urdf"
        )

        super().__init__(
            links,
            name=name,
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
            gripper_links=links[7]
        )

        self.manufacturer = "MERA"

        # ready pose, arm up
        self.qr = np.array([0, pi / 2, -pi / 2, 0, 0, 0])
        self.qz = np.zeros(6)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)

        # zero angles, upper arm horizontal, lower up straight up
        self.addconfiguration_attr("qz", np.array([0, 0, 0, 0, 0, 0]))

if __name__ == "__main__":  # pragma nocover

    robot = X_5002()
    print(robot)

    print(robot.ee_links)
