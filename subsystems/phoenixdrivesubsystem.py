from commands2 import Subsystem
from phoenix6.hardware import Pigeon2
from constants import TunerConstants, AutoConstants
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.controller import PIDController
from wpimath.kinematics import SwerveModulePosition, ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics
from subsystems.phoenixswervemodule import PhoenixSwerveModule
from wpilib import Timer, DriverStation, Field2d, SmartDashboard
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
from pathplannerlib.commands import FollowPathHolonomic
from math import pi


class PhoenixDriveSubsystem(Subsystem):

    def __init__(self, timer: Timer):
        super().__init__()

        # Create a local reference for the master clock.
        self.timer = timer

        # Create a local storage variable for Alliance state (this is in case we lose connection.)
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.blue_alliance = False
        else:
            self.blue_alliance = True

        # Instantiate the Pigeon.
        self.gyro = Pigeon2(TunerConstants.gyro_id, TunerConstants.k_can_bus_name)

        # Create the pose estimator object.
        self.odometry = SwerveDrive4PoseEstimator(
            TunerConstants.kinematics,
            Rotation2d.fromDegrees(self.get_heading()),
            (SwerveModulePosition(0, Rotation2d(0)),
             SwerveModulePosition(0, Rotation2d(0)),
             SwerveModulePosition(0, Rotation2d(0)),
             SwerveModulePosition(0, Rotation2d(0))),
            Pose2d(Translation2d(0, 0), Rotation2d(0)))

        # Instantiate all four modules.
        self.fl_mod = PhoenixSwerveModule(TunerConstants.k_front_left_drive_motor_id,
                                          TunerConstants.k_front_left_steer_motor_id,
                                          TunerConstants.k_front_left_encoder_id,
                                          TunerConstants.k_front_left_drive_motor_inverted,
                                          TunerConstants.k_front_left_steer_motor_inverted)
        self.fr_mod = PhoenixSwerveModule(TunerConstants.k_front_right_drive_motor_id,
                                          TunerConstants.k_front_right_steer_motor_id,
                                          TunerConstants.k_front_right_encoder_id,
                                          TunerConstants.k_front_right_drive_motor_inverted,
                                          TunerConstants.k_front_right_steer_motor_inverted)
        self.bl_mod = PhoenixSwerveModule(TunerConstants.k_back_left_drive_motor_id,
                                          TunerConstants.k_back_left_steer_motor_id,
                                          TunerConstants.k_back_left_encoder_id,
                                          TunerConstants.k_back_left_drive_motor_inverted,
                                          TunerConstants.k_back_left_steer_motor_inverted)
        self.br_mod = PhoenixSwerveModule(TunerConstants.k_back_right_drive_motor_id,
                                          TunerConstants.k_back_right_steer_motor_id,
                                          TunerConstants.k_back_right_encoder_id,
                                          TunerConstants.k_back_right_drive_motor_inverted,
                                          TunerConstants.k_back_right_steer_motor_inverted)

        # Reset drive encoders to ensure we have a clean startup.
        self.reset_encoders()

        # Create an object to store target module states.
        self.target_states = [
            SwerveModuleState(0, Rotation2d(0)),
            SwerveModuleState(0, Rotation2d(0)),
            SwerveModuleState(0, Rotation2d(0)),
            SwerveModuleState(0, Rotation2d(0))
        ]

        # Prepare onboard PID controllers.
        self.clt_controller = PIDController(TunerConstants.clt_controller_PID[0],
                                            TunerConstants.clt_controller_PID[1],
                                            TunerConstants.clt_controller_PID[2])
        self.snap_controller = PIDController(TunerConstants.snap_controller_PID[0],
                                             TunerConstants.snap_controller_PID[1],
                                             TunerConstants.snap_controller_PID[2])
        self.snap_controller.enableContinuousInput(-180, 180)

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

        self.field = Field2d()
        self.debug_mode = False
        self.last_time = self.timer.get()
        self.current_time = self.timer.get()
        self.periodic_limiter_timer = self.timer.get()

    def get_chassis_speeds(self) -> ChassisSpeeds:
        """Acquire ChassisSpeeds object from current module states."""
        return TunerConstants.kinematics.toChassisSpeeds((self.fl_mod.get_state(), self.fr_mod.get_state(),
                                                          self.bl_mod.get_state(), self.br_mod.get_state()))

    def drive_by_chassis_speeds(self, input_speeds: ChassisSpeeds) -> None:
        """Command the robot to drive using a ChassisSpeeds object."""
        module_states = TunerConstants.kinematics.toSwerveModuleStates(input_speeds)

        self.fl_mod.set_desired_state(module_states[0])
        self.fr_mod.set_desired_state(module_states[1])
        self.bl_mod.set_desired_state(module_states[2])
        self.br_mod.set_desired_state(module_states[3])

    def get_heading(self) -> float:
        """Return robot yaw as a float."""
        return self.gyro.get_yaw().value_as_double

    def get_heading_odo(self) -> Rotation2d:
        """Return robot yaw as a Rotation2d from the pose estimator."""
        return self.odometry.getEstimatedPosition().rotation()

    def reset_encoders(self) -> None:
        """Reset all drive encoders. DOES NOT INCLUDE STEER ENCODERS."""
        self.fl_mod.reset_drive_encoder()
        self.fr_mod.reset_drive_encoder()
        self.bl_mod.reset_drive_encoder()
        self.br_mod.reset_drive_encoder()

    def get_pose(self) -> Pose2d:
        """Return pose from the pose estimator."""
        return self.odometry.getEstimatedPosition()

    def reset_odometry(self, pose: Pose2d) -> None:
        """Reset robot odometry. Set pose to an input location."""
        self.reset_encoders()
        self.odometry.resetPosition(Rotation2d.fromDegrees(self.get_heading()),
                                    (
                                        SwerveModulePosition(0, self.fl_mod.get_position().angle),
                                        SwerveModulePosition(0, self.fr_mod.get_position().angle),
                                        SwerveModulePosition(0, self.bl_mod.get_position().angle),
                                        SwerveModulePosition(0, self.br_mod.get_position().angle)
                                    ),
                                    pose)

    def get_path_flip(self) -> bool:
        """Check if autonomous path needs to be mirrored."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return True
        else:
            return False

    def set_module_states(self, desired_states) -> None:
        # Update the local object that stores target states.
        self.target_states = desired_states

        # Desaturate wheel speeds.
        adjusted_speeds = SwerveDrive4Kinematics.desaturateWheelSpeeds(desired_states,
                                                                       TunerConstants.k_speed_at_12_volts_Mps)

        # Set module states.
        self.fl_mod.set_desired_state(adjusted_speeds[0])
        self.fr_mod.set_desired_state(adjusted_speeds[1])
        self.bl_mod.set_desired_state(adjusted_speeds[2])
        self.br_mod.set_desired_state(adjusted_speeds[3])

    def drive_basic(self, x_speed: float, y_speed: float, rotation: float, field_relative: bool) -> None:
        """Basic drive command."""
        # Update current time.
        self.current_time = self.timer.get()

        # If on the blue alliance, mirror inputs.
        if self.blue_alliance:
            x_speed = -1 * x_speed
            y_speed = -1 * y_speed

        # Generate target module states using the inputs.
        if not field_relative:
            module_states = TunerConstants.kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(x_speed, y_speed, -rotation, self.current_time - self.last_time))
        else:
            module_states = TunerConstants.kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                    ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, -rotation, self.get_heading_odo()),
                    self.current_time - self.last_time
                )
            )

        # Set each module state.
        self.set_module_states(module_states)

        # Update the "last_time" value to keep track of loop cycles.
        self.last_time = self.timer.get()

    def drive_closed_loop_turning(self, x_speed: float, y_speed: float, rotation: float, scale: float,
                                  damper: float) -> None:
        """Drive in teleop with closed loop turning enabled."""
        current_heading = self.get_heading_odo().degrees()
        dmp = 1 - (rotation * damper)
        heading_target = current_heading + scale * rotation
        rotate_output = self.clt_controller.calculate(heading_target, current_heading)
        self.drive_basic(x_speed * dmp, y_speed * dmp, rotate_output, True)

    def drive_slow_clt(self, x_speed: float, y_speed: float, rotation: float, scale: float, damper: float,
                       slow: float) -> None:
        """Drive in closed loop turning mode at a reduced speed."""
        self.drive_closed_loop_turning(x_speed * slow, y_speed * slow, rotation * slow, scale, damper)

    def parking_brake(self) -> None:
        """Lock the wheels in an X pattern."""
        self.set_module_states([SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                               SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                               SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                               SwerveModuleState(0, Rotation2d.fromDegrees(45))])

    def add_vision(self, pose: Pose2d, timestamp: float, stdevs: []) -> None:
        """Add a vision measurement to the pose estimator."""
        self.odometry.addVisionMeasurement(pose, timestamp, stdevs)

    def snap_drive(self, x_speed: float, y_speed: float, heading_target: float) -> None:
        """Lock robot heading to a specified orientation."""
        current_heading = self.get_heading_odo().degrees()
        if self.blue_alliance:
            heading_target = heading_target + 180
        rotate_output = self.snap_controller.calculate(heading_target, current_heading)
        self.drive_basic(x_speed, y_speed, rotate_output, True)

    def return_wheels_to_zero(self) -> None:
        """Set wheels to 0 and/or 180 degree angles."""
        self.drive_basic(0.01, 0, 0, False)

    def debug_switch(self, on: bool) -> None:
        """Function to make toggling Debug Mode easier."""
        self.debug_mode = on

    def follow_path_command(self, target_coordinates: [], end_rotation: float) -> FollowPathHolonomic:
        """Generate and follow a PathPlanner path in real time."""
        # Create object to store points needed to generate bezier curve.
        bezier_points = PathPlannerPath.bezierFromPoses([
            self.get_pose(),
            Pose2d(target_coordinates[0], target_coordinates[1], Rotation2d.fromDegrees(target_coordinates[2]))
        ])

        # Generate path.
        path = PathPlannerPath(
            bezier_points,
            PathConstraints(2, 1.5, 2 * pi, 4 * pi),
            GoalEndState(0, Rotation2d.fromDegrees(end_rotation))
        )

        # Return command to follow the generated path.
        return FollowPathHolonomic(
            path,
            self.get_pose,
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

    def periodic(self) -> None:
        """Update robot odometry, pose, dashboard, etc."""
        # Update robot pose as fast as possible, accounting for time delay.
        self.odometry.updateWithTime(self.timer.get(),
                                     Rotation2d.fromDegrees(self.get_heading()),
                                     (
                                         self.fl_mod.get_position(),
                                         self.fr_mod.get_position(),
                                         self.bl_mod.get_position(),
                                         self.br_mod.get_position()
                                     ))

        # Set the robot pose on the Field Widget.
        self.field.setRobotPose(self.get_pose())

        # Create a slower periodic loop to do CPU intensive items.
        if self.timer.get() - 1 > self.periodic_limiter_timer:
            # Make sure the alliance hasn't changed recently.
            if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                self.blue_alliance = True
            else:
                self.blue_alliance = False

            self.periodic_limiter_timer = self.timer.get()

        # Deploy critical information to NetworkTables.
        SmartDashboard.putData("Field", self.field)
        SmartDashboard.putNumber("Current Heading", self.get_heading_odo().degrees())
        SmartDashboard.putString("Estimated Pose", str(self.get_pose()))

        # Deploy information to NetworkTables that is available in Debug Mode.
        if self.debug_mode:
            SmartDashboard.putNumber("FL Angle", self.fl_mod.get_state().angle.degrees())
            SmartDashboard.putNumber("FL Speed", abs(self.fl_mod.get_state().speed))
            SmartDashboard.putNumber("FL Target Angle", self.target_states[0].angle.degrees())
            SmartDashboard.putNumber("FL Target Speed", self.target_states[0].speed)

            SmartDashboard.putNumber("FR Angle", self.fr_mod.get_state().angle.degrees())
            SmartDashboard.putNumber("FR Speed", abs(self.fr_mod.get_state().speed))
            SmartDashboard.putNumber("FR Target Angle", self.target_states[1].angle.degrees())
            SmartDashboard.putNumber("FR Target Speed", self.target_states[1].speed)

            SmartDashboard.putNumber("BL Angle", self.bl_mod.get_state().angle.degrees())
            SmartDashboard.putNumber("BL Speed", abs(self.bl_mod.get_state().speed))
            SmartDashboard.putNumber("BL Target Angle", self.target_states[2].angle.degrees())
            SmartDashboard.putNumber("BL Target Speed", self.target_states[2].speed)

            SmartDashboard.putNumber("BR Angle", self.br_mod.get_state().angle.degrees())
            SmartDashboard.putNumber("BR Speed", abs(self.br_mod.get_state().speed))
            SmartDashboard.putNumber("BR Target Angle", self.target_states[3].angle.degrees())
            SmartDashboard.putNumber("BR Target Speed", self.target_states[3].speed)

            SmartDashboard.putNumber("IMU Yaw", self.get_heading())
            SmartDashboard.putNumber("IMU Roll", self.gyro.get_roll().value_as_double)
            SmartDashboard.putNumber("IMU Pitch", self.gyro.get_pitch().value_as_double)
