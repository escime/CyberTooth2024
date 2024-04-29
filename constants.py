"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""

import math
from wpimath.geometry import Translation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians


class OIConstants:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1


class VisionConstants:
    rotation_from_horizontal = 30.5  # In degrees.
    lens_height = 20.52718  # In inches.
    tag_heights = [52.625, 52.625, 56.375, 56.375, 52.625, 52.625, 56.375, 56.375,
                   52.625, 52.625, 52, 52, 52, 52, 52, 52]  # In inches.
    turnkP = 0.055  # Will require tuning. # 0.05
    turnkI = 0
    turnkD = 0  # was 0.5
    rangekP = 0.05
    turn_to_target_error_max = 2  # In degrees.
    min_command = 0.09  # Should be in volts, will require tuning.
    shooter_default_speed = 5300  # 4500
    speaker_location_blue = [0, 5.53]
    speaker_location_red = [16.5, 5.53]
    mp_ttt_kp = 0.08
    mp_ttt_ki = 0
    mp_ttt_kd = 0.1
