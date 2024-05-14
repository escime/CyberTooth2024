import math
from phoenix5.sensors import Pigeon2
import commands2
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModulePosition
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.controller import PIDController
from subsystems.swervemodule import SwerveModule
from constants import DriveConstants, ModuleConstants, AutoConstants
from wpilib import SmartDashboard, Field2d, Timer, DriverStation
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
from pathplannerlib.commands import FollowPathHolonomic
from helpers.pose_estimator import PoseEstimator


class DriveSubsystem(commands2.Subsystem):
    # Creates a new DriveSubsystem
    def __init__(self, timer: Timer, pose_estimator: PoseEstimator) -> None:
        super().__init__()
        self.gyro = Pigeon2(9)
        self.pose_estimator = pose_estimator

        # Reset odometry @ instantiation.
        self.gyro.setYaw(180)

        # Setup snap controller for class-wide use.
        self.snap_controller = PIDController(DriveConstants.snap_controller_PID[0],
                                             DriveConstants.snap_controller_PID[1],
                                             DriveConstants.snap_controller_PID[2])

        self.snap_controller.enableContinuousInput(-180, 180)

        self.turret_controller = PIDController(DriveConstants.turret_controller_PID[0],
                                               DriveConstants.turret_controller_PID[1],
                                               DriveConstants.turret_controller_PID[2])
        self.turret_controller.enableContinuousInput(-180, 180)

        # Setup closed-loop-turning controller
        self.clt_controller = PIDController(DriveConstants.clt_controller_PID[0],
                                            DriveConstants.clt_controller_PID[1],
                                            DriveConstants.clt_controller_PID[2])
        self.clt_controller.enableContinuousInput(0, 360)

        # Setup controller for auto-balance.
        self.balance_controller = PIDController(DriveConstants.balance_PID[0],
                                                DriveConstants.balance_PID[1],
                                                DriveConstants.balance_PID[2])

        # Setup a couple indicators.
        self.balanced = True
        self.debug_mode = False

        # Instantiate all swerve modules.
        self.m_FL = SwerveModule(ModuleConstants.fl_drive_id,
                                 ModuleConstants.fl_turn_id,
                                 ModuleConstants.fl_encoder_id,
                                 ModuleConstants.fl_zero_offset,
                                 False,
                                 True)
        self.m_FR = SwerveModule(ModuleConstants.fr_drive_id,
                                 ModuleConstants.fr_turn_id,
                                 ModuleConstants.fr_encoder_id,
                                 ModuleConstants.fr_zero_offset,
                                 False,
                                 True)
        self.m_BL = SwerveModule(ModuleConstants.bl_drive_id,
                                 ModuleConstants.bl_turn_id,
                                 ModuleConstants.bl_encoder_id,
                                 ModuleConstants.bl_zero_offset,
                                 False,
                                 True)
        self.m_BR = SwerveModule(ModuleConstants.br_drive_id,
                                 ModuleConstants.br_turn_id,
                                 ModuleConstants.br_encoder_id,
                                 ModuleConstants.br_zero_offset,
                                 False,
                                 True)

        # Set initial value of software-tracked position.
        self.m_FL_position = self.m_FL.get_position_onboard()
        self.m_FR_position = self.m_FR.get_position_onboard()
        self.m_BL_position = self.m_BL.get_position_onboard()
        self.m_BR_position = self.m_BR.get_position_onboard()

        self.reset_encoders()

        AutoBuilder.configureHolonomic(
            self.get_pose,
            self.reset_odometry,
            self.get_chassis_speeds,
            self.drive_by_chassis_speeds,
            HolonomicPathFollowerConfig(
                PIDConstants(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController),
                PIDConstants(AutoConstants.kPThetaController, 0, 0),
                AutoConstants.max_module_speed,
                AutoConstants.module_radius_from_center,
                ReplanningConfig()
            ),
            self.get_path_flip,
            self
        )

        self.blue_alliance = False

        self.timer = timer
        self.last_time = 0
        self.period_update_time = self.timer.get()
        self.current_time = self.timer.get()

        # Setup for shoot while moving calculations
        self.loop_time = self.timer.get()
        self.vx_old = 0
        self.vy_old = 0
        self.omega_old = 0
        self.vx_new = 0
        self.vy_new = 0
        self.omega_new = 0
        self.ax = 0
        self.ay = 0
        self.alpha = 0

    # Create Field2d object to display/track robot position.
    m_field = Field2d()

    def get_chassis_speeds(self):
        """Used for 2024 PathPlanner. Converts current module states to a ChassisSpeeds object."""
        return DriveConstants.m_kinematics.toChassisSpeeds((self.m_FL.get_state_onboard(),
                                                           self.m_FR.get_state_onboard(),
                                                           self.m_BL.get_state_onboard(),
                                                           self.m_BR.get_state_onboard()))

    def drive_by_chassis_speeds(self, chassis_speeds: ChassisSpeeds):
        """Used for 2024 PathPlanner. Takes in ChassisSpeeds, sets target module states."""
        swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(chassis_speeds)

        # Set all swerve module state targets and update the dashboard with the targets.
        self.m_FL.set_desired_state_onboard(swerve_module_states[0])
        self.m_FR.set_desired_state_onboard(swerve_module_states[1])
        self.m_BL.set_desired_state_onboard(swerve_module_states[2])
        self.m_BR.set_desired_state_onboard(swerve_module_states[3])

    def drive_2ok(self, x_speed: float, y_speed: float, rot: float, field_relative: bool) -> None:
        """
        Drive the robot with second order kinematics enabled.
        x_speed: Float, -max_speed to +max_speed.
        y_speed: Float, -max_speed to +max_speed.
        rot: Float, -max_angular_speed to + max_angular_speed.
        field_relative: Boolean. Toggles between field relative and robot relative.
        """
        self.current_time = self.timer.get()
        if self.blue_alliance:
            x_speed = -1 * x_speed
            y_speed = -1 * y_speed
        if not field_relative:
            swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(x_speed, y_speed, -rot, self.current_time - self.last_time)
            )
        else:
            swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                    ChassisSpeeds.fromFieldRelativeSpeeds(x_speed,
                                                          y_speed,
                                                          -rot,
                                                          self.get_heading_odo()),
                    self.current_time - self.last_time
                )
            )

        swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, DriveConstants.kMaxSpeed)

        self.set_module_states(swerve_module_states)

        if self.debug_mode:
            if swerve_module_states[0].angle.degrees() >= 0:
                SmartDashboard.putNumber("FL Target", swerve_module_states[0].angle.degrees())
            else:
                SmartDashboard.putNumber("FL Target", (swerve_module_states[0].angle.degrees() + 360))
            SmartDashboard.putNumber("FL Target Speed", swerve_module_states[0].speed)
            if swerve_module_states[1].angle.degrees() >= 0:
                SmartDashboard.putNumber("FR Target", swerve_module_states[1].angle.degrees())
            else:
                SmartDashboard.putNumber("FR Target", (swerve_module_states[1].angle.degrees() + 360))
            SmartDashboard.putNumber("FR Target Speed", swerve_module_states[1].speed)
            if swerve_module_states[2].angle.degrees() >= 0:
                SmartDashboard.putNumber("BL Target", swerve_module_states[2].angle.degrees())
            else:
                SmartDashboard.putNumber("BL Target", (swerve_module_states[2].angle.degrees() + 360))
            SmartDashboard.putNumber("BL Target Speed", swerve_module_states[2].speed)
            if swerve_module_states[3].angle.degrees() >= 0:
                SmartDashboard.putNumber("BR Target", swerve_module_states[3].angle.degrees())
            else:
                SmartDashboard.putNumber("BR Target", (swerve_module_states[3].angle.degrees() + 360))
            SmartDashboard.putNumber("BR Target Speed", swerve_module_states[3].speed)

        self.last_time = self.timer.get()

    def drive_2ok_clt(self, x_speed: float, y_speed: float, rot: float, scale: float):
        current_heading = self.get_heading_odo().degrees()
        # Rot is a value between -1 and 1
        # when scale is 1, the robot rotates by 1 degree when the stick is fully TURNT
        heading_target = current_heading + scale * rot
        rotate_output = self.clt_controller.calculate(heading_target, current_heading)
        self.drive_2ok(x_speed, y_speed, rotate_output, True)

    def drive_2ok_clt_dmp(self, x_speed: float, y_speed: float, rot: float, scale: float, damping_scalar: float):
        damp = 1 - (rot * damping_scalar)
        start_time = self.timer.get()
        self.drive_2ok_clt(x_speed * damp, y_speed * damp, rot, scale)
        SmartDashboard.putNumber("Drive Function Runtime", self.timer.get() - start_time)

    def drive(self, x_speed: float, y_speed: float, rot: float, field_relative: bool) -> None:
        """The default drive command for the robot.
        x_speed: Float, -max_speed to +max_speed.
        y_speed: Float, -max_speed to +max_speed.
        rot: Float, -max_angular_speed to + max_angular_speed.
        field_relative: Boolean. Toggles between field relative and robot relative.
        """
        # If in field relative mode, get swerve module states.
        if field_relative:
            swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, -rot,
                                                      self.get_heading_odo()))
        # If in robot relative mode, get swerve module states.
        else:
            swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(ChassisSpeeds(-x_speed,
                                                                                                  -y_speed, -rot))

        # Desaturate wheel speeds step based on max robot speed.
        swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, DriveConstants.kMaxSpeed)

        # Set all swerve module state targets and update the dashboard with the targets.
        self.m_FL.set_desired_state_onboard(swerve_module_states[0])
        self.m_FR.set_desired_state_onboard(swerve_module_states[1])
        self.m_BL.set_desired_state_onboard(swerve_module_states[2])
        self.m_BR.set_desired_state_onboard(swerve_module_states[3])

        # if self.debug_mode is True:
        if self.debug_mode:
            if swerve_module_states[0].angle.degrees() >= 0:
                SmartDashboard.putNumber("FL Target", swerve_module_states[0].angle.degrees())
            else:
                SmartDashboard.putNumber("FL Target", (swerve_module_states[0].angle.degrees() + 360))
            SmartDashboard.putNumber("FL Target Speed", swerve_module_states[0].speed)
            if swerve_module_states[1].angle.degrees() >= 0:
                SmartDashboard.putNumber("FR Target", swerve_module_states[1].angle.degrees())
            else:
                SmartDashboard.putNumber("FR Target", (swerve_module_states[1].angle.degrees() + 360))
            SmartDashboard.putNumber("FR Target Speed", swerve_module_states[1].speed)
            if swerve_module_states[2].angle.degrees() >= 0:
                SmartDashboard.putNumber("BL Target", swerve_module_states[2].angle.degrees())
            else:
                SmartDashboard.putNumber("BL Target", (swerve_module_states[2].angle.degrees() + 360))
            SmartDashboard.putNumber("BL Target Speed", swerve_module_states[2].speed)
            if swerve_module_states[3].angle.degrees() >= 0:
                SmartDashboard.putNumber("BR Target", swerve_module_states[3].angle.degrees())
            else:
                SmartDashboard.putNumber("BR Target", (swerve_module_states[3].angle.degrees() + 360))
            SmartDashboard.putNumber("BR Target Speed", swerve_module_states[3].speed)

    def drive_slow(self, x_speed: float, y_speed: float, rot: float, field_relative: bool, slow: float) -> None:
        """Alternate drive command that reduces maximum speed by a given multiplier.
        x_speed: Float, -max_speed to +max_speed.
        y_speed: Float, -max_speed to +max_speed.
        rot: Float, -max_angular_speed to + max_angular_speed.
        field_relative: Boolean. Toggles between field relative and robot relative.
        slow: FLoat, 0 to 1, multiplier for speed decrease.
        """
        self.drive_2ok(x_speed * slow, y_speed * slow, rot * slow, field_relative)

    def drive_lock(self) -> None:
        """Alternate drive command that locks all swerve modules into rotation position, a 'hard' brake setting."""
        swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
            ChassisSpeeds(0, 0, 0.01))

        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, DriveConstants.kMaxSpeed)

        self.m_FL.set_desired_state_onboard(swerve_module_states[0])
        self.m_FR.set_desired_state_onboard(swerve_module_states[1])
        self.m_BL.set_desired_state_onboard(swerve_module_states[2])
        self.m_BR.set_desired_state_onboard(swerve_module_states[3])

    def periodic(self):
        """Update robot odometry, pose, and dashboard readouts."""
        start_time = self.timer.get()
        self.pose_estimator.update_odometry(Rotation2d.fromDegrees(self.get_heading()),
                                            self.m_FL.get_position_onboard(),
                                            self.m_FR.get_position_onboard(),
                                            self.m_BL.get_position_onboard(),
                                            self.m_BR.get_position_onboard())
        self.m_field.setRobotPose(self.get_pose())

        self.vx_new, self.vy_new, self.omega_new = self.get_field_relative_velocity()
        self.ax, self.ay, self.alpha = self.get_field_relative_acceleration([self.vx_new, self.vy_new, self.omega_new],
                                                                            [self.vx_old, self.vy_old, self.omega_old],
                                                                            self.timer.get() - self.loop_time)
        self.vx_old = self.vx_new
        self.vy_old = self.vy_new
        self.omega_old = self.omega_new
        self.loop_time = self.timer.get()

        if self.timer.get() - 0.5 > self.period_update_time:
            if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                self.blue_alliance = True
            else:
                self.blue_alliance = False
            # if -5 < self.gyro.getRoll() < 5:
            #     self.balanced = True
            # else:
            #     self.balanced = False
            self.period_update_time = self.timer.get()

        SmartDashboard.putData("Field", self.m_field)
        SmartDashboard.putNumber("Current Odo Heading", self.get_heading_odo().degrees())

        # SmartDashboard.putNumber("Robot Heading", self.get_heading())
        # SmartDashboard.putNumber("Robot Pitch", self.gyro.getRoll())
        # SmartDashboard.putBoolean("Balanced?", self.balanced)
        SmartDashboard.putString("Estimated Pose", str(self.get_pose()))

        SmartDashboard.putNumber("Drive Periodic Runtime", self.timer.get() - start_time)

        if self.debug_mode is True:
            SmartDashboard.putNumber("FL Angle", self.m_FL.get_state_onboard().angle.degrees())
            SmartDashboard.putNumber("FL Speed", abs(self.m_FL.get_state_onboard().speed))
            SmartDashboard.putNumber("FR Angle", self.m_FR.get_state_onboard().angle.degrees())
            SmartDashboard.putNumber("FR Speed", abs(self.m_FR.get_state_onboard().speed))
            SmartDashboard.putNumber("BL Angle", self.m_BL.get_state_onboard().angle.degrees())
            SmartDashboard.putNumber("BL Speed", abs(self.m_BL.get_state_onboard().speed))
            SmartDashboard.putNumber("BR Angle", self.m_BR.get_state_onboard().angle.degrees())
            SmartDashboard.putNumber("BR Speed", abs(self.m_BR.get_state_onboard().speed))
            SmartDashboard.putData("Snap Controller", self.snap_controller)
            SmartDashboard.putData("CLT Controller", self.clt_controller)

    def get_pose(self):
        """Return pose estimator's estimated position."""
        return self.pose_estimator.get_pose()

    def reset_odometry(self, pose: Pose2d):
        """
        Hard reset robot odometry and pose. Intended only for manual use.
        pose: Pose2D object. Pose you would like to set the robot's position to.
        """
        self.m_FL.reset_encoders()
        self.m_FR.reset_encoders()
        self.m_BL.reset_encoders()
        self.m_BR.reset_encoders()
        self.pose_estimator.reset_odometry(pose,
                                           Rotation2d.fromDegrees(self.get_heading()),
                                           SwerveModulePosition(0, self.m_FL.get_position_onboard().angle),
                                           SwerveModulePosition(0, self.m_FR.get_position_onboard().angle),
                                           SwerveModulePosition(0, self.m_BL.get_position_onboard().angle),
                                           SwerveModulePosition(0, self.m_BR.get_position_onboard().angle))

    def reset_odo_and_gyro(self, pose: Pose2d):
        if self.get_path_flip():
            self.gyro.setYaw(180)
        else:
            self.gyro.setYaw(0)
        self.reset_odometry(pose)

    def reset_position_no_rotation(self, location: Translation2d):
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            current_rotation = Rotation2d.fromDegrees(-1 * self.get_heading() + 180)
        else:
            current_rotation = Rotation2d.fromDegrees(-1 * self.get_heading())
        self.m_FL.reset_encoders()
        self.m_FR.reset_encoders()
        self.m_BL.reset_encoders()
        self.m_BR.reset_encoders()
        self.pose_estimator.reset_odometry(Pose2d(location, current_rotation),
                                           Rotation2d.fromDegrees(self.get_heading()),
                                           SwerveModulePosition(0, self.m_FL.get_position_onboard().angle),
                                           SwerveModulePosition(0, self.m_FR.get_position_onboard().angle),
                                           SwerveModulePosition(0, self.m_BL.get_position_onboard().angle),
                                           SwerveModulePosition(0, self.m_BR.get_position_onboard().angle))

    def set_module_states(self, desired_states):
        """Set swerve module states given a list of target states. Used to simplify drive_2ok."""
        SwerveDrive4Kinematics.desaturateWheelSpeeds(desired_states, DriveConstants.kMaxSpeed)
        self.m_FL.set_desired_state_onboard(desired_states[0])
        self.m_FR.set_desired_state_onboard(desired_states[1])
        self.m_BL.set_desired_state_onboard(desired_states[2])
        self.m_BR.set_desired_state_onboard(desired_states[3])

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
        return self.gyro.getYaw()

    def get_heading_odo(self):
        return self.pose_estimator.get_heading()

    def snap_drive(self, x_speed: float, y_speed: float, heading_target: float):
        """
        Calculate and implement the PID controller for rotating to and maintaining a target heading.
        x_speed: Float, -max_speed to +max_speed.
        y_speed: Float, -max_speed to +max_speed.
        heading_target: Float, degree target angle.
        """
        current_heading = self.get_heading_odo().degrees()
        if self.blue_alliance:
            heading_target = heading_target + 180
        rotate_output = self.snap_controller.calculate(heading_target, current_heading)
        self.drive_2ok(x_speed, y_speed, rotate_output, True)

    def snap_drive_absolute(self, x_speed: float, y_speed: float, heading_target: float):
        """
        Maintain a target heading on the field absolute scale.
        x_speed: Float, -max_speed to +max_speed
        y_speed: Float, -max_speed to +max_speed
        heading_target; Float, degree target angle
        """
        self.drive_2ok(x_speed, y_speed, self.snap_controller.calculate(heading_target,
                                                                        self.get_heading_odo().degrees()), True)

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

    def get_path_flip(self) -> bool:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return True
        else:
            return False

    def follow_path_command(self, target_location: [], end_rotation: float) -> FollowPathHolonomic:
        bezier_points = PathPlannerPath.bezierFromPoses([
            self.get_pose(),
            Pose2d(target_location[0], target_location[1], Rotation2d.fromDegrees(target_location[2]))])

        path = PathPlannerPath(
            bezier_points,
            PathConstraints(2, 2, 2 * 3.14159, 4 * 3.14159),
            GoalEndState(0, Rotation2d.fromDegrees(end_rotation))
        )

        return FollowPathHolonomic(
            path,
            self.get_pose,
            self.get_chassis_speeds,
            self.drive_by_chassis_speeds,
            HolonomicPathFollowerConfig(
                PIDConstants(AutoConstants.kPXController, 0, 0),
                PIDConstants(AutoConstants.kPThetaController, 0, 0),
                AutoConstants.max_module_speed,
                AutoConstants.module_radius_from_center,
                ReplanningConfig()
            ),
            self.get_path_flip,
            self
        )

    def get_field_relative_velocity(self):
        return self.get_chassis_speeds().vx * self.get_heading_odo().cos() - \
               self.get_chassis_speeds().vy * self.get_heading_odo().sin(), \
               self.get_chassis_speeds().vy * self.get_heading_odo().cos() + \
               self.get_chassis_speeds().vx * self.get_heading_odo().sin(), self.get_chassis_speeds().omega

    def get_field_relative_acceleration(self, new_speed, old_speed, time: float):
        ax = (new_speed[0] - old_speed[0]) / time
        ay = (new_speed[1] - old_speed[1]) / time
        alpha = (new_speed[2] - old_speed[2]) / time

        if abs(ax) > 6.0:
            ax = 6.0 * math.copysign(1, ax)
        if abs(ay) > 6.0:
            ay = 6.0 * math.copysign(1, ay)
        if abs(alpha) > 4 * math.pi:
            alpha = 4 * math.pi * math.copysign(1, alpha)

        return ax, ay, alpha

    def turret_drive(self, x_speed: float, y_speed: float, heading_target: float):
        """
        Calculate and implement the PID controller for rotating to and maintaining a target heading.
        x_speed: Float, -max_speed to +max_speed.
        y_speed: Float, -max_speed to +max_speed.
        heading_target: Float, degree target angle.
        """
        current_heading = self.get_heading_odo().degrees()
        if self.blue_alliance:
            heading_target = heading_target + 180
        rotate_output = self.turret_controller.calculate(heading_target, current_heading)
        self.drive_2ok(x_speed, y_speed, rotate_output, True)

    def get_current_draw_all_modules(self) -> [[float, float], [float, float], [float, float], [float, float]]:
        return [
            self.m_FL.get_current_draw(),
            self.m_FR.get_current_draw(),
            self.m_BL.get_current_draw(),
            self.m_BR.get_current_draw()
        ]
