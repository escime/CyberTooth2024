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
from pathplannerlib.commands import FollowPathHolonomic, PathfindHolonomic
from pathplannerlib.logging import PathPlannerLogging
from helpers.pose_estimator import PoseEstimator


class DriveSubsystem(commands2.Subsystem):
    # Creates a new DriveSubsystem
    def __init__(self, timer: Timer, pose_estimator: PoseEstimator) -> None:
        super().__init__()
        self.gyro = Pigeon2(9)
        self.pose_estimator = pose_estimator

        # Setup "snap" controller, which is used to point the robot to a given heading.
        self.snap_controller = PIDController(DriveConstants.snap_controller_PID[0],
                                             DriveConstants.snap_controller_PID[1],
                                             DriveConstants.snap_controller_PID[2])
        self.snap_controller.enableContinuousInput(-180, 180)

        # Setup "turret" controller, which is similar to the snap controller, but has different parameters because it
        # is used only in shooting while moving code.
        self.turret_controller = PIDController(DriveConstants.turret_controller_PID[0],
                                               DriveConstants.turret_controller_PID[1],
                                               DriveConstants.turret_controller_PID[2])
        self.turret_controller.enableContinuousInput(-180, 180)

        # Setup "closed-loop-turning controller" which is used to maintain robot heading while translating.
        self.clt_controller = PIDController(DriveConstants.clt_controller_PID[0],
                                            DriveConstants.clt_controller_PID[1],
                                            DriveConstants.clt_controller_PID[2])
        self.clt_controller.enableContinuousInput(0, 360)

        # Setup "balance" controller which was used to balance the robot on the Charge Station in Charged Up.
        self.balance_controller = PIDController(DriveConstants.balance_PID[0],
                                                DriveConstants.balance_PID[1],
                                                DriveConstants.balance_PID[2])
        self.balanced = True

        # Instantiate all swerve modules by passing the proper IDs and offsets to them.
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

        # Set initial value of software-tracked position. All of these should be zero, but sometimes SPARK MAXes are
        # weird, so we take this safety step.
        self.m_FL_position = self.m_FL.get_position_onboard()
        self.m_FR_position = self.m_FR.get_position_onboard()
        self.m_BL_position = self.m_BL.get_position_onboard()
        self.m_BR_position = self.m_BR.get_position_onboard()

        # Resets module drive encoders to prep for the rest of startup.
        # TODO Check if this is redundant code.
        self.reset_encoders()

        # Prepare the autobuilder package from PathPlanner to run autonomous.
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

        # Setup a boolean that tracks if "debug mode" is enabled, which allows the drivetrain to spew significantly
        # more data than normal to the dashboard.
        self.debug_mode = False

        # Setup a boolean to locally store which alliance the robot is on. The system will periodically check, but this
        # ensures that we are never in a situation where a read error will prevent the robot from functioning.
        self.blue_alliance = False

        # Import the master timer and set up a couple of timecodes for use later.
        self.timer = timer
        self.last_time = 0
        self.period_update_time = self.timer.get()
        self.current_time = self.timer.get()

        # Setup for shoot while moving calculations, this is all for determining the robot's average speed and
        # acceleration.
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

        # Setup some variables for Closed Loop Turning.
        self.clt_target = self.pose_estimator.get_heading().degrees()
        self.clt_first_time = True
        self.clt_reset = False
        self.last_clt_call = self.timer.get()

        # Create Field2d object to display/track robot position.
        self.m_field = Field2d()

    def get_chassis_speeds(self):
        """Used for 2024 PathPlanner. Converts current module states to a ChassisSpeeds object."""
        return DriveConstants.m_kinematics.toChassisSpeeds((self.m_FL.get_state_onboard(),
                                                           self.m_FR.get_state_onboard(),
                                                           self.m_BL.get_state_onboard(),
                                                           self.m_BR.get_state_onboard()))

    def drive_by_chassis_speeds(self, chassis_speeds: ChassisSpeeds):
        """Used for 2024 PathPlanner. Takes in ChassisSpeeds, sets target module states."""
        swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(chassis_speeds)

        self.set_module_states(swerve_module_states)

        # Set all swerve module state targets and update the dashboard with the targets.
        # self.m_FL.set_desired_state_onboard(swerve_module_states[0])
        # self.m_FR.set_desired_state_onboard(swerve_module_states[1])
        # self.m_BL.set_desired_state_onboard(swerve_module_states[2])
        # self.m_BR.set_desired_state_onboard(swerve_module_states[3])
        self.clt_target = self.get_heading_odo().degrees()

    def drive_2ok(self, x_speed: float, y_speed: float, rot: float, field_relative: bool) -> None:
        """
        Drive the robot with loop time variance correction.
        x_speed: Float, -max_speed to +max_speed.
        y_speed: Float, -max_speed to +max_speed.
        rot: Float, -max_angular_speed to + max_angular_speed.
        field_relative: Boolean. Toggles between field relative and robot relative.
        """
        # Get the start time of the function run.
        self.current_time = self.timer.get()

        # Check if the robot speed needs to be inverted.
        if self.blue_alliance:
            x_speed = -1 * x_speed
            y_speed = -1 * y_speed

        # If in robot relative, drive robot relative.
        # TODO Check if the blue alliance inversion should happen after this smh
        if not field_relative:
            swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(x_speed, y_speed, -rot, self.current_time - self.last_time)
            )
        # If in field relative, drive in field relative.
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

        # Set the module states.
        self.set_module_states(swerve_module_states)

        # Record the last time this function ran.
        self.last_time = self.timer.get()

    def drive_2ok_clt(self, x_speed: float, y_speed: float, rot: float, field_relative: bool) -> None:
        """Drive with closed loop turning and loop variance compensation active."""
        current_heading = self.get_heading_odo().degrees()

        if self.clt_first_time:
            self.clt_target = current_heading
            self.clt_first_time = False

        if rot != 0:
            self.drive_2ok(x_speed, y_speed, rot * DriveConstants.kMaxAngularSpeed, field_relative)
            self.clt_reset = True
        elif x_speed <= 0.25 * DriveConstants.kMaxSpeed and y_speed <= 0.25 * DriveConstants.kMaxSpeed:
            self.drive_2ok(x_speed, y_speed, rot * DriveConstants.kMaxAngularSpeed, field_relative)
            self.clt_reset = True
        else:
            if self.clt_reset:
                current_angular_velocity = self.get_angular_velocity() * 180 / math.pi
                self.clt_target = current_heading + (current_angular_velocity * (self.timer.get() - self.last_clt_call))
                self.clt_reset = False
            rotate_output = self.clt_controller.calculate(self.clt_target, current_heading)
            self.drive_2ok(x_speed, y_speed, rotate_output, field_relative)

        self.last_clt_call = self.timer.get()

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

        # Set all swerve module state targets and update the dashboard with the targets.
        self.set_module_states(swerve_module_states)

    def drive_slow(self, x_speed: float, y_speed: float, rot: float, field_relative: bool, slow: float) -> None:
        """Alternate drive command that reduces maximum speed by a given multiplier.
        x_speed: Float, -max_speed to +max_speed.
        y_speed: Float, -max_speed to +max_speed.
        rot: Float, -max_angular_speed to + max_angular_speed.
        field_relative: Boolean. Toggles between field relative and robot relative.
        slow: FLoat, 0 to 1, multiplier for speed decrease.
        """
        self.drive_2ok_clt(x_speed * slow, y_speed * slow, rot * slow, field_relative)

    def drive_lock(self) -> None:
        """Alternate drive command that locks all swerve modules into rotation position, a 'hard' brake setting."""
        swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
            ChassisSpeeds(0, 0, 0.01))

        self.set_module_states(swerve_module_states)

    def get_pose(self) -> Pose2d:
        """Return pose estimator's estimated position."""
        return self.pose_estimator.get_pose()

    def reset_odometry(self, pose: Pose2d) -> None:
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

    def reset_position_no_rotation(self, location: Translation2d) -> None:
        """For manually performing a hard pose update that won't affect the robot's rotation. WARNING: Deprecated."""
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

    def set_module_states(self, desired_states) -> None:
        """Set swerve module states given a list of target states. If in debug mode, inform the dashboard of the
        targeted states."""
        desired_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(desired_states, DriveConstants.kMaxSpeed)
        self.m_FL.set_desired_state_onboard(desired_states[0])
        self.m_FR.set_desired_state_onboard(desired_states[1])
        self.m_BL.set_desired_state_onboard(desired_states[2])
        self.m_BR.set_desired_state_onboard(desired_states[3])

        if self.debug_mode:
            if desired_states[0].angle.degrees() >= 0:
                SmartDashboard.putNumber("FL Target", desired_states[0].angle.degrees())
            else:
                SmartDashboard.putNumber("FL Target", (desired_states[0].angle.degrees() + 360))
            SmartDashboard.putNumber("FL Target Speed", desired_states[0].speed)
            if desired_states[1].angle.degrees() >= 0:
                SmartDashboard.putNumber("FR Target", desired_states[1].angle.degrees())
            else:
                SmartDashboard.putNumber("FR Target", (desired_states[1].angle.degrees() + 360))
            SmartDashboard.putNumber("FR Target Speed", desired_states[1].speed)
            if desired_states[2].angle.degrees() >= 0:
                SmartDashboard.putNumber("BL Target", desired_states[2].angle.degrees())
            else:
                SmartDashboard.putNumber("BL Target", (desired_states[2].angle.degrees() + 360))
            SmartDashboard.putNumber("BL Target Speed", desired_states[2].speed)
            if desired_states[3].angle.degrees() >= 0:
                SmartDashboard.putNumber("BR Target", desired_states[3].angle.degrees())
            else:
                SmartDashboard.putNumber("BR Target", (desired_states[3].angle.degrees() + 360))
            SmartDashboard.putNumber("BR Target Speed", desired_states[3].speed)

    def reset_encoders(self) -> None:
        """Manually reset only the swerve module drive encoders."""
        self.m_FL.reset_encoders()
        self.m_FR.reset_encoders()
        self.m_BL.reset_encoders()
        self.m_BR.reset_encoders()

    def zero_heading(self) -> None:
        """Reset robot absolute heading to zero. WARNING: DO NOT USE."""
        self.gyro.setYaw(0)

    def get_heading(self) -> Rotation2d:
        """Retrieve robot heading directly from the IMU."""
        return self.gyro.getYaw()

    def get_heading_odo(self) -> Rotation2d:
        """Returns the pose estimator's estimated robot heading."""
        return self.pose_estimator.get_heading()

    def snap_drive(self, x_speed: float, y_speed: float, heading_target: float) -> None:
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
        self.clt_target = current_heading
        self.drive_2ok(x_speed, y_speed, rotate_output, True)

    def snap_drive_targeting(self, x_speed: float, y_speed: float, heading_target: float) -> None:
        current_heading = self.get_heading_odo().degrees()
        if self.blue_alliance:
            heading_target = heading_target + 180
        rotate_output = self.snap_controller.calculate(heading_target, current_heading)
        if -0.15 <= rotate_output <= 0:
            rotate_output = -0.09
        elif 0 < rotate_output <= 0.15:
            rotate_output = 0.09
        self.clt_target = self.get_heading_odo().degrees()
        self.drive_2ok(x_speed, y_speed, rotate_output, True)

    def snap_drive_absolute(self, x_speed: float, y_speed: float, heading_target: float) -> None:
        """
        Maintain a target heading on the field absolute scale.
        x_speed: Float, -max_speed to +max_speed
        y_speed: Float, -max_speed to +max_speed
        heading_target; Float, degree target angle
        """
        self.clt_target = self.get_heading_odo().degrees()
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

    def debug_toggle(self, on: bool) -> None:
        """Toggles Debug Mode on and off."""
        self.debug_mode = on

    def get_path_flip(self) -> bool:
        """Returns true if the autonomous paths need to be mirrored."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return True
        else:
            return False

    def follow_path_command(self, target_location: []) -> FollowPathHolonomic:
        """Transforms a set of target coordinates and an end rotation state into a parth following command."""
        bezier_points = PathPlannerPath.bezierFromPoses([
            self.get_pose(),
            Pose2d(target_location[0], target_location[1], Rotation2d.fromDegrees(target_location[2]))])

        path = PathPlannerPath(
            bezier_points,
            PathConstraints(2, 2, 2 * 3.14159, 4 * 3.14159),
            GoalEndState(0, Rotation2d.fromDegrees(target_location[2]))
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

    def pathfind(self, target_location: []):
        target_pose = Pose2d(target_location[0], target_location[1], Rotation2d.fromDegrees(target_location[2]))

        constraints = PathConstraints(3, 4, 9.424, 12.567)

        return PathfindHolonomic(
            constraints,
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
            lambda: False,
            self,
            rotation_delay_distance=0,
            target_pose=target_pose,
            goal_end_vel=0
        )

    def get_field_relative_velocity(self) -> [float, float]:
        """Returns the instantaneous velocity of the robot."""
        return self.get_chassis_speeds().vx * self.get_heading_odo().cos() - \
            self.get_chassis_speeds().vy * self.get_heading_odo().sin(), \
            self.get_chassis_speeds().vy * self.get_heading_odo().cos() + \
            self.get_chassis_speeds().vx * self.get_heading_odo().sin(), self.get_chassis_speeds().omega

    def get_angular_velocity(self) -> float:
        return self.get_chassis_speeds().omega

    def get_field_relative_acceleration(self, new_speed, old_speed, time: float) -> [float, float, float]:
        """Returns the instantaneous acceleration of the robot."""
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

    def turret_drive(self, x_speed: float, y_speed: float, heading_target: float) -> None:
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
        self.clt_target = current_heading
        self.drive_2ok(x_speed, y_speed, rotate_output, True)

    def get_current_draw_all_modules(self) -> [[float, float], [float, float], [float, float], [float, float]]:
        """Returns the current draws for each swerve module as a list."""
        return [
            self.m_FL.get_current_draw(),
            self.m_FR.get_current_draw(),
            self.m_BL.get_current_draw(),
            self.m_BR.get_current_draw()
        ]

    def periodic(self) -> None:
        """Update robot odometry, pose, and dashboard readouts."""
        # Record the time that periodic begins.
        start_time = self.timer.get()

        # Update the pose estimator and pose display with any changes in robot state.
        self.pose_estimator.update_odometry(Rotation2d.fromDegrees(self.get_heading()),
                                            self.m_FL.get_position_onboard(),
                                            self.m_FR.get_position_onboard(),
                                            self.m_BL.get_position_onboard(),
                                            self.m_BR.get_position_onboard())
        self.m_field.setRobotPose(self.get_pose())
        PathPlannerLogging.setLogActivePathCallback(lambda poses: self.m_field.getObject('path').setPoses(poses))

        # Record the robot's instantaneous velocity and acceleration.
        self.vx_new, self.vy_new, self.omega_new = self.get_field_relative_velocity()
        self.ax, self.ay, self.alpha = self.get_field_relative_acceleration([self.vx_new, self.vy_new, self.omega_new],
                                                                            [self.vx_old, self.vy_old, self.omega_old],
                                                                            self.timer.get() - self.loop_time)
        self.vx_old = self.vx_new
        self.vy_old = self.vy_new
        self.omega_old = self.omega_new
        self.loop_time = self.timer.get()

        # Perform any low time priority tasks.
        if self.timer.get() - 0.5 > self.period_update_time:
            # Check if the robot's alliance is the same as it was on startup.
            if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                self.blue_alliance = True
            else:
                self.blue_alliance = False

            self.period_update_time = self.timer.get()

            SmartDashboard.putData("Field", self.m_field)
        SmartDashboard.putNumber("Current Odo Heading", self.get_heading_odo().degrees())
        # SmartDashboard.putNumber("CLT Target Heading", self.clt_target)
        # SmartDashboard.putString("Estimated Pose", str(self.get_pose()))
        # TODO See how much the SmartDashboard pushes are slowing down drive periodic.
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
