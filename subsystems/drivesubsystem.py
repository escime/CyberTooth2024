from rev import CANSparkMax
from phoenix5.sensors import CANCoder, Pigeon2
import commands2
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.controller import PIDController
from subsystems.swervemodule import SwerveModule
from constants import DriveConstants, ModuleConstants, AutoConstants
from wpilib import SmartDashboard, Field2d, Timer
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants


class DriveSubsystem(commands2.SubsystemBase):
    # Creates a new DriveSubsystem
    def __init__(self) -> None:
        super().__init__()
        self.m_odometry = SwerveDrive4PoseEstimator(DriveConstants.m_kinematics,
                                                    Rotation2d.fromDegrees(-self.get_heading()),
                                                    (SwerveModulePosition(0, Rotation2d(0)),
                                                     SwerveModulePosition(0, Rotation2d(0)),
                                                     SwerveModulePosition(0, Rotation2d(0)),
                                                     SwerveModulePosition(0, Rotation2d(0))),
                                                    Pose2d(Translation2d(0, 0), Rotation2d(0)))

        # Reset odometry @ instantiation.
        self.gyro.setYaw(0)

        # Setup snap controller for class-wide use.
        self.snap_controller = PIDController(DriveConstants.snap_controller_PID[0],
                                             DriveConstants.snap_controller_PID[1],
                                             DriveConstants.snap_controller_PID[2])
        self.snap_controller.enableContinuousInput(-180, 180)

        # Setup controller for auto-balance.
        self.balance_controller = PIDController(DriveConstants.balance_PID[0],
                                                DriveConstants.balance_PID[1],
                                                DriveConstants.balance_PID[2])

        # Setup a couple indicators.
        self.balanced = True
        self.debug_mode = False

        # Instantiate all swerve modules.
        self.m_FL = SwerveModule(CANSparkMax(ModuleConstants.fl_drive_id, CANSparkMax.MotorType.kBrushless),
                                 CANSparkMax(ModuleConstants.fl_turn_id, CANSparkMax.MotorType.kBrushless),
                                 CANCoder(ModuleConstants.fl_encoder_id),
                                 ModuleConstants.fl_zero_offset,
                                 True,
                                 True)
        self.m_FR = SwerveModule(CANSparkMax(ModuleConstants.fr_drive_id, CANSparkMax.MotorType.kBrushless),
                                 CANSparkMax(ModuleConstants.fr_turn_id, CANSparkMax.MotorType.kBrushless),
                                 CANCoder(ModuleConstants.fr_encoder_id),
                                 ModuleConstants.fr_zero_offset,
                                 True,
                                 True)
        self.m_BL = SwerveModule(CANSparkMax(ModuleConstants.bl_drive_id, CANSparkMax.MotorType.kBrushless),
                                 CANSparkMax(ModuleConstants.bl_turn_id, CANSparkMax.MotorType.kBrushless),
                                 CANCoder(ModuleConstants.bl_encoder_id),
                                 ModuleConstants.bl_zero_offset,
                                 True,
                                 True)
        self.m_BR = SwerveModule(CANSparkMax(ModuleConstants.br_drive_id, CANSparkMax.MotorType.kBrushless),
                                 CANSparkMax(ModuleConstants.br_turn_id, CANSparkMax.MotorType.kBrushless),
                                 CANCoder(ModuleConstants.br_encoder_id),
                                 ModuleConstants.br_zero_offset,
                                 True,
                                 True)

        # Set initial value of software-tracked position.
        self.m_FL_position = self.m_FL.get_position()
        self.m_FR_position = self.m_FR.get_position()
        self.m_BL_position = self.m_BL.get_position()
        self.m_BR_position = self.m_BR.get_position()

        self.reset_encoders()

        # TODO Check functionality. Concerned about commanded speed in simulator.
        AutoBuilder.configureHolonomic(
            self.get_pose,
            self.reset_odometry,
            self.get_chassis_speeds,
            self.drive_by_chassis_speeds,
            HolonomicPathFollowerConfig(
                PIDConstants(AutoConstants.kPXController, 0, 0),
                PIDConstants(AutoConstants.kPThetaController, 0, 0),
                AutoConstants.max_module_speed,
                AutoConstants.module_radius_from_center,
                ReplanningConfig()
            ),
            self
        )

    # Instantiate gyro.
    gyro = Pigeon2(9)

    # Create Field2d object to display/track robot position.
    m_field = Field2d()

    # Start timer for 2nd order kinematics.
    # TODO Disable this if 2ok is not being used.
    timer = Timer()
    timer.start()
    last_time = 0
    current_time = timer.get()

    def get_chassis_speeds(self):
        """Used for 2024 PathPlanner."""
        return DriveConstants.m_kinematics.toChassisSpeeds((self.m_FL.get_state(),
                                                           self.m_FR.get_state(),
                                                           self.m_BL.get_state(),
                                                           self.m_BR.get_state()))

    def drive_by_chassis_speeds(self, chassis_speeds: ChassisSpeeds):
        """Used for 2024 PathPlanner."""
        swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(chassis_speeds)

        # Set all swerve module state targets and update the dashboard with the targets.
        self.m_FL.set_desired_state(swerve_module_states[0])
        self.m_FR.set_desired_state(swerve_module_states[1])
        self.m_BL.set_desired_state(swerve_module_states[2])
        self.m_BR.set_desired_state(swerve_module_states[3])

    def drive_2ok(self, x_speed: float, y_speed: float, rot: float, field_relative: bool) -> None:
        """Drive the robot with second order kinematics enabled."""
        self.current_time = self.timer.get()
        if not field_relative:
            swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(-x_speed, -y_speed, -rot, self.current_time - self.last_time)
            )
        else:
            swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                    ChassisSpeeds.fromFieldRelativeSpeeds(-x_speed,
                                                          -y_speed,
                                                          -rot,
                                                          Rotation2d.fromDegrees(-self.get_heading())),
                    self.current_time - self.last_time
                )
            )
            self.set_module_states(swerve_module_states)
            self.last_time = self.timer.get()

    def drive(self, x_speed, y_speed, rot, field_relative) -> None:
        """The default drive command for the robot. This is the math that makes swerve drive work."""
        # If in field relative mode, get swerve module states.
        if field_relative:
            swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(-x_speed, -y_speed, -rot,
                                                      Rotation2d.fromDegrees(-self.get_heading())))
        # If in robot relative mode, get swerve module states.
        else:
            swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(ChassisSpeeds(-x_speed,
                                                                                                  -y_speed, -rot))

        # Desaturate wheel speeds step based on max robot speed.
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, DriveConstants.kMaxSpeed)

        # Set all swerve module state targets and update the dashboard with the targets.
        self.m_FL.set_desired_state(swerve_module_states[0])
        self.m_FR.set_desired_state(swerve_module_states[1])
        self.m_BL.set_desired_state(swerve_module_states[2])
        self.m_BR.set_desired_state(swerve_module_states[3])

        # if self.debug_mode is True:
        if self.debug_mode:
            SmartDashboard.putNumber("FL Target", swerve_module_states[0].angle.degrees())
            SmartDashboard.putNumber("FL Target Speed", swerve_module_states[0].speed)
            SmartDashboard.putNumber("FR Target", swerve_module_states[1].angle.degrees())
            SmartDashboard.putNumber("FR Target Speed", swerve_module_states[1].speed)
            SmartDashboard.putNumber("BL Target", swerve_module_states[2].angle.degrees())
            SmartDashboard.putNumber("BL Target Speed", swerve_module_states[2].speed)
            SmartDashboard.putNumber("BR Target", swerve_module_states[3].angle.degrees())
            SmartDashboard.putNumber("BR Target Speed", swerve_module_states[3].speed)

    def drive_slow(self, x_speed, y_speed, rot, field_relative, slow: float) -> None:
        """Alternate drive command that reduces maximum speed by a given multiplier."""
        self.drive(x_speed * slow, y_speed * slow, rot * slow, field_relative)

    def drive_lock(self) -> None:
        """Alternate drive command that locks all swerve modules into rotation position, a 'hard' brake setting."""
        swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
            ChassisSpeeds(0, 0, 0.01))

        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, DriveConstants.kMaxSpeed)

        self.m_FL.set_desired_state(swerve_module_states[0])
        self.m_FR.set_desired_state(swerve_module_states[1])
        self.m_BL.set_desired_state(swerve_module_states[2])
        self.m_BR.set_desired_state(swerve_module_states[3])

    def periodic(self):
        """Update robot odometry, pose, and dashboard readouts."""
        self.m_odometry.update(Rotation2d.fromDegrees(-self.get_heading()),
                               (self.m_FL.get_position(),
                                self.m_FR.get_position(),
                                self.m_BL.get_position(),
                                self.m_BR.get_position()))
        self.m_field.setRobotPose(self.get_pose())
        if -5 < self.gyro.getRoll() < 5:
            self.balanced = True
        else:
            self.balanced = False

        SmartDashboard.putData("Field", self.m_field)
        SmartDashboard.putNumber("Robot Heading", self.get_heading())
        SmartDashboard.putNumber("Robot Pitch", self.gyro.getRoll())
        SmartDashboard.putBoolean("Balanced?", self.balanced)
        SmartDashboard.putString("Estimated Pose", str(self.get_pose()))

        if self.debug_mode is True:
            SmartDashboard.putNumber("FL Angle", self.m_FL.get_state().angle.degrees())
            SmartDashboard.putNumber("FL Speed", self.m_FL.get_state().speed)
            SmartDashboard.putNumber("FR Angle", self.m_FR.get_state().angle.degrees())
            SmartDashboard.putNumber("FR Speed", self.m_FR.get_state().speed)
            SmartDashboard.putNumber("BL Angle", self.m_BL.get_state().angle.degrees())
            SmartDashboard.putNumber("BL Speed", self.m_BL.get_state().speed)
            SmartDashboard.putNumber("BR Angle", self.m_BR.get_state().angle.degrees())
            SmartDashboard.putNumber("BR Speed", self.m_BR.get_state().speed)
            SmartDashboard.putString("Current Command", str(self.getCurrentCommand()))

    def get_pose(self):
        """Return pose estimator's estimated position."""
        return Pose2d(self.m_odometry.getEstimatedPosition().x, self.m_odometry.getEstimatedPosition().y,
                      self.m_odometry.getEstimatedPosition().rotation())

    def add_vision(self, pose: Pose2d, timestamp: float):
        """Add a vision measurement from the limelight and integrate into robot pose using a Kalman filter."""
        self.m_odometry.addVisionMeasurement(pose, timestamp)

    def reset_odometry(self, pose: Pose2d):
        """Hard reset robot odometry and pose. Intended only for manual use."""
        self.m_FL.reset_encoders()
        self.m_FR.reset_encoders()
        self.m_BL.reset_encoders()
        self.m_BR.reset_encoders()
        self.zero_heading()
        self.gyro.setYaw(pose.rotation().degrees())
        self.m_odometry.resetPosition(Rotation2d.fromDegrees(-self.get_heading()),
                                      (SwerveModulePosition(0, self.m_FL_position.angle),
                                       SwerveModulePosition(0, self.m_FR_position.angle),
                                       SwerveModulePosition(0, self.m_BL_position.angle),
                                       SwerveModulePosition(0, self.m_BR_position.angle)),
                                      pose)

    def set_module_states(self, desired_states):
        """Set swerve module states given a list of target states. Used to simplify drive_2ok."""
        SwerveDrive4Kinematics.desaturateWheelSpeeds(desired_states, DriveConstants.kMaxSpeed)
        self.m_FL.set_desired_state(desired_states[0])
        self.m_FR.set_desired_state(desired_states[1])
        self.m_BL.set_desired_state(desired_states[2])
        self.m_BR.set_desired_state(desired_states[3])

    def reset_encoders(self):
        """Manually reset only the swerve module encoders."""
        self.m_FL.reset_encoders()
        self.m_FR.reset_encoders()
        self.m_BL.reset_encoders()
        self.m_BR.reset_encoders()

    def zero_heading(self):
        """Reset robot absolute heading to zero."""
        self.gyro.setYaw(0)

    def get_heading(self):
        """Retrieve robot heading from the IMU."""
        return -1 * self.gyro.getYaw()

    def snap_drive(self, x_speed: float, y_speed: float, heading_target: float):
        """Calculate and implement the PID controller for rotating to and maintaining a target heading."""
        current_heading = Rotation2d.fromDegrees(self.get_heading()).degrees() % 180
        correction = Rotation2d.fromDegrees(self.get_heading()).degrees() % 360
        if correction > 180:
            current_heading = current_heading - 180
        rotate_output = self.snap_controller.calculate(current_heading, heading_target)
        self.drive(x_speed, y_speed, rotate_output, True)

    def auto_balance(self, front_back: int):
        """Automatically balance on the charge station. front_back = 1 for forward. -1 for backward."""
        balance_output = self.balance_controller.calculate(-1 * self.gyro.getRoll(), 0)
        if self.balanced is False:
            self.snap_drive(DriveConstants.kMaxSpeed * front_back * balance_output, 0, 0)
        else:
            self.drive_lock()

    def return_wheels_to_zero(self) -> None:
        """Set wheels to known forward direction for auto startup."""
        self.drive(0.01, 0, 0, False)

    def set_start_position(self, angle: int, x_pos: float, y_pos: float) -> None:
        """Reset pose to the location an autonomous mode starts from."""
        self.reset_odometry(Pose2d(x_pos, y_pos, Rotation2d.fromDegrees(angle)))

    def debug_toggle(self, on: bool):
        self.debug_mode = on
