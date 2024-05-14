from wpimath.estimator import SwerveDrive4PoseEstimator
from constants import DriveConstants
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.kinematics import SwerveModulePosition


class PoseEstimator:
    def __init__(self):
        super().__init__()
        self.estimator = SwerveDrive4PoseEstimator(DriveConstants.m_kinematics,
                                                   Rotation2d.fromDegrees(0),
                                                   (SwerveModulePosition(0, Rotation2d(0)),
                                                    SwerveModulePosition(0, Rotation2d(0)),
                                                    SwerveModulePosition(0, Rotation2d(0)),
                                                    SwerveModulePosition(0, Rotation2d(0))),
                                                   Pose2d(Translation2d(1.31, 5.54), Rotation2d(0)))

        self.estimator.setVisionMeasurementStdDevs((0.2, 0.2, 999999999))

    def update_odometry(self, rotation: Rotation2d, fl_pos: SwerveModulePosition, fr_pos: SwerveModulePosition,
                        bl_pos: SwerveModulePosition, br_pos: SwerveModulePosition) -> None:
        """Update the pose estimator for the drivetrain."""
        self.estimator.update(rotation, (fl_pos, fr_pos, bl_pos, br_pos))

    def get_pose(self) -> Pose2d:
        """Return the current estimated position of the drivetrain."""
        return self.estimator.getEstimatedPosition()

    def add_vision(self, pose: Pose2d, timestamp: float):
        """Add a pose recorded via vision to the pose estimator's Kalman Filter."""
        self.estimator.addVisionMeasurement(pose, timestamp)

    def reset_odometry(self, target_pose: Pose2d, current_angle: Rotation2d, fl_pos: SwerveModulePosition,
                       fr_pos: SwerveModulePosition, bl_pos: SwerveModulePosition,
                       br_pos: SwerveModulePosition) -> None:
        """Reset the pose estimator for the drivetrain."""
        self.estimator.resetPosition(current_angle, (fl_pos, fr_pos, bl_pos, br_pos), target_pose)

    def get_heading(self) -> Rotation2d:
        """Return the current robot orientation."""
        return self.estimator.getEstimatedPosition().rotation()
