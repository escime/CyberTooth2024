"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""

import math
from wpimath.geometry import Translation2d, Pose2d
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
    # kMaxSpeedTeleop = kMaxSpeed * 1.75
    kMaxAngularSpeed = 7  # Set max rotation speed rot/s 20
    # kMaxAngularSpeedTeleop = kMaxAngularSpeed * 1.75
    kGyroReversed = False

    m_FL_location = Translation2d(0.52705 / 2, 0.52705 / 2)
    m_FR_location = Translation2d(0.52705 / 2, -0.52705 / 2)
    m_BL_location = Translation2d(-0.52705 / 2, 0.52705 / 2)
    m_BR_location = Translation2d(-0.52705 / 2, -0.52705 / 2)
    m_kinematics = SwerveDrive4Kinematics(m_FL_location, m_FR_location, m_BL_location, m_BR_location)

    snap_controller_PID = [0.05, 0, 0]  # 0.05
    turret_controller_PID = [0.08, 0, 0.0001]
    clt_controller_PID = [1, 0, 0]
    drive_controller_PID = [2, 0, 0]
    azimuth_controller_PID = [1.8, 0, 0]
    drive_controller_FF = [0.18 / 12, 2.35, 0.44]  # n/a, 2.35, 0.44

    closed_loop_ramp = 0.0
    open_loop_ramp = 0.25
    drive_current_limit = 60  # Was 38
    azimuth_current_limit = 30  # Was 38

    balance_PID = [0.01, 0, 0]

    slew_rate_drive = 130  # 50  # 110
    slew_rate_turn = 0


class AutoConstants:
    kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeed
    kMaxAccelerationMetersPerSecondSquared = 0.5

    kPXController = 5  # Previously 5
    kIXController = 0
    kDXController = 0
    kPThetaController = 5  # Previously 3
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(kMaxSpeedMetersPerSecond,
                                                                      kMaxAccelerationMetersPerSecondSquared)
    max_module_speed = kMaxSpeedMetersPerSecond * 0.8
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
    kP = 1
    arm_limit = 60
    climb_limit = 60
    trap_speed = 0.5
    shoot_speed = 0.5
    amp_speed = 0.4
    climber_preset = 270
    climber_preset_2 = 270
    sport_reduction = 1/16
    sprocket_reduction = 10/46
    neo_resolution = 1/42
    positionConversion = 360 * neo_resolution * sprocket_reduction * sport_reduction
    current_threshold = 30


class IntakeConstants:
    current_limit = 60
    motor_id = 30
    follower_id = 31
    intake_speed = 1


class VisionConstants:
    rotation_from_horizontal = 30.5  # In degrees.
    # rotation_from_horizontal = 45  # TODO Correct here and in the limelight GUI
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


class ShooterConstants:
    top_id = 34
    bottom_id = 33
    current_limit = 38
    shooter_kFF = 0.000165  # 0.000172
    shooter_kP = 0.0002  # 0.0002
    shooter_kD = 0
    angle_kP = 10.5  # 4
    angle_kI = 0.00015
    feeder_speed = 0.9
    threshold = 200  # 500
    threshold_ang = 0.003  # 0.01
    threshold_fired = 30
    trim = -0.005


class GlobalVariables:
    # This really shouldn't go here, but I'm trying to fix something really bad. This is also very stupid.
    current_vision = Pose2d(0, 0, 0)
    timestamp = 0
