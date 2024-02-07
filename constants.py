"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""

import math
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians


class DriveConstants:
    wheel_diameter = 0.1016  # meters
    wheel_circumference = wheel_diameter * math.pi
    drive_gear_ratio = 6.12
    angle_gear_ratio = 21.43

    # d_velocity_conversion_factor = 0.0007885761
    # d_position_conversion_factor = 0.047314566  # L2 ratio is 6.746031745
    d_position_conversion_factor = wheel_circumference / drive_gear_ratio
    d_velocity_conversion_factor = d_position_conversion_factor / 60  # Conversion from rot/min to m/s
    kMaxSpeed = 5.06  # Set max speed in m/s 10
    kMaxSpeedTeleop = kMaxSpeed * 1.75
    kMaxAngularSpeed = 11  # Set max rotation speed rot/s 20
    kMaxAngularSpeedTeleop = kMaxAngularSpeed * 1.75
    kGyroReversed = False

    m_FL_location = Translation2d(0.52705 / 2, 0.52705 / 2)
    m_FR_location = Translation2d(0.52705 / 2, -0.52705 / 2)
    m_BL_location = Translation2d(-0.52705 / 2, 0.52705 / 2)
    m_BR_location = Translation2d(-0.52705 / 2, -0.52705 / 2)
    m_kinematics = SwerveDrive4Kinematics(m_FL_location, m_FR_location, m_BL_location, m_BR_location)

    snap_controller_PID = [0, 0, 0]
    drive_controller_PID = [2, 0, 0]
    azimuth_controller_PID = [2, 0, 0]
    drive_controller_FF = [0.20 / 12, 2.35 / 12, 0.44 / 12]  # n/a, 2.35, 0.44

    closed_loop_ramp = 0.0
    open_loop_ramp = 0.25
    drive_current_limit = 60  # Was 38
    azimuth_current_limit = 30  # Was 38

    balance_PID = [0.01, 0, 0]

    slew_rate_drive = 40
    slew_rate_turn = 0


class AutoConstants:
    kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeed * 0.5
    kMaxAccelerationMetersPerSecondSquared = 2

    kPXController = 5  # Previously 12
    kPThetaController = 7  # Previously 10
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(kMaxSpeedMetersPerSecond,
                                                                      kMaxAccelerationMetersPerSecondSquared)
    max_module_speed = kMaxSpeedMetersPerSecond
    module_distance = 0.52705  # in m
    # module_radius_from_center = math.sqrt(pow((module_distance / 2), 2) + pow((module_distance / 2), 2))
    module_radius_from_center = 0.372681


class OIConstants:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1


class ModuleConstants:
    fl_drive_id = 10
    fl_turn_id = 11
    fl_encoder_id = 12
    fl_zero_offset = -248.91

    fr_drive_id = 13
    fr_turn_id = 14
    fr_encoder_id = 15
    fr_zero_offset = -223.33

    br_drive_id = 19
    br_turn_id = 20
    br_encoder_id = 21
    br_zero_offset = -53.53

    bl_drive_id = 16
    bl_turn_id = 17
    bl_encoder_id = 18
    bl_zero_offset = -202.85


class TrapperConstants:
    kP = 0.05
    arm_limit = 60
    climb_limit = 60
    trap_speed = 0.5
    amp_speed = 1
    climber_preset = 0
    climber_preset_2 = 0
    sport_reduction = 1/16
    sprocket_reduction = 10/46
    neo_resolution = 1/42
    positionConversion = 360 * neo_resolution * sprocket_reduction * sport_reduction


class IntakeConstants:
    current_limit = 35
    motor_id = 30
    follower_id = 31
    intake_speed = 1


class VisionConstants:
    rotation_from_horizontal = 30.5  # In degrees.
    lens_height = 20.52718  # In inches.
    tag_heights = [52.625, 52.625, 56.375, 56.375, 52.625, 52.625, 56.375, 56.375,
                   52.625, 52.625, 52, 52, 52, 52, 52, 52]  # In inches.
    turnkP = 0.01  # Will require tuning.
    rangekP = 0.05
    turn_to_target_error_max = 2  # In degrees.
    min_command = 0.01  # Should be in volts, will require tuning.
    shooter_default_speed = 3000


class ShooterConstants:
    master_id = 33
    follower_id = 34
    current_limit = 38
    shooter_kFF = 0  # estimated 0.19 Vs/m
    shooter_kP = 0
    shooter_kD = 0
    angle_kP = 0
    feeder_speed = 0
    threshold = 50
    threshold_ang = 1
    threshold_fired = 30
    trim = 0
