from wpimath.controller import PIDController, ProfiledPIDControllerRadians
import math
import commands2
import commands2.cmd
from constants import AutoConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.ledsubsystem import LEDs
from subsystems.visionsubsystem import VisionSubsystem
from commands.return_wheels import ReturnWheels
from commands.turn import Turn
from commands.vision_estimate import VisionEstimate
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory


def follow_trajectory(traj: Trajectory, first_path: bool, drive: DriveSubsystem) -> \
        commands2.SequentialCommandGroup:
    """Return automated command to follow a generated trajectory."""
    theta_controller = ProfiledPIDControllerRadians(AutoConstants.kPThetaController, 0, 0,
                                                    AutoConstants.kThetaControllerConstraints, 0.02)
    theta_controller.enableContinuousInput(-math.pi, math.pi)
    if first_path:
        drive.reset_odometry(traj.initialPose())
    scc = commands2.Swerve4ControllerCommand(traj,
                                             drive.get_pose,
                                             DriveConstants.m_kinematics,
                                             PIDController(AutoConstants.kPXController, 0, 0),
                                             PIDController(AutoConstants.kPYController, 0, 0),
                                             theta_controller,
                                             drive.set_module_states,
                                             [drive]
                                             )
    return commands2.SequentialCommandGroup(
        scc,
        commands2.cmd.runOnce(lambda: drive.drive(0, 0, 0, False), [drive])
    )


def generate_wpi_trajectory(start: Pose2d, waypoints: [Translation2d], end: Pose2d, vmax: float, amax: float) \
        -> Trajectory:
    """Generates a wpi trajectory from a starting pose, list of waypoints, ending pose, and constraints."""
    return TrajectoryGenerator.generateTrajectory(start, waypoints, end, TrajectoryConfig(vmax, amax))


def map_to_red(x: float, y: float, theta: float, mapp: bool) -> [float, float, float]:
    """Maps a pose from the blue side of the field to the red side of the field."""
    if mapp:
        return Pose2d(16.45 - x, y, Rotation2d().fromDegrees(-1 * theta))
    else:
        return Pose2d(x, y, Rotation2d().fromDegrees(theta))


def map_to_red_trans(x: float, y: float, mapp: bool) -> [float, float]:
    """Maps a translation from the blue side of the field to the red side of the field."""
    if mapp:
        return Translation2d(16.45 - x, y)
    else:
        return Translation2d(x, y)


def gen_and_run(start: Pose2d, waypoints: [Translation2d], end: Pose2d, vmax: float, amax: float, first_path: bool,
                drive: DriveSubsystem) -> commands2.SequentialCommandGroup:
    """Generates a path following command based on the constraints for a path given."""
    return follow_trajectory(generate_wpi_trajectory(start, waypoints, end, vmax, amax), first_path, drive)


def AUTO_test_commands(vision: VisionSubsystem, drive: DriveSubsystem, leds: LEDs) -> commands2.SequentialCommandGroup:
    return commands2.SequentialCommandGroup(
        commands2.WaitCommand(1)
    )


def AUTO_reset_with_vision(vision: VisionSubsystem, drive: DriveSubsystem) -> commands2.SequentialCommandGroup:
    return commands2.SequentialCommandGroup(
        VisionEstimate(vision, drive)
    )


def AUTO_path_tuning(drive: DriveSubsystem, leds: LEDs) -> commands2.SequentialCommandGroup:
    return commands2.SequentialCommandGroup(
        ReturnWheels(drive),
        commands2.ParallelRaceGroup(
            gen_and_run(map_to_red(0, 0, 0, False), [], map_to_red(1, -1, 180, False), 3, 2, True, drive),
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), [leds])
        )
    )
