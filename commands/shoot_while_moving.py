from commands2 import Command, InterruptionBehavior
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from constants import VisionConstants, DriveConstants
from wpilib import Timer, DriverStation
from wpimath.geometry import Translation2d
from helpers.custom_hid import CustomHID
from math import sqrt, pow


class ShootVisionWhileMoving(Command):
    def __init__(self, shooter: ShooterSubsystem, vision: VisionSubsystem, intake: IntakeSubsystem,
                 trapper: TrapperSubsystem, drive: DriveSubsystem, timer: Timer, controller: CustomHID, scale: float,
                 speed_scalar: float):
        super().__init__()
        self.shooter = shooter
        self.vision = vision
        self.drive = drive
        self.controller = controller
        self.scale = scale
        self.intake = intake
        self.trapper = trapper
        self.speed_scalar = speed_scalar
        self.addRequirements(intake)
        self.addRequirements(trapper)
        self.addRequirements(shooter)
        self.addRequirements(drive)
        self.timer = timer
        self.overrun_time = self.timer.get()
        self.time_to_shot = 1  # Really this should be a lookup table but I'll get there
        self.accel_comp = 0.1  # Unmeasurable garbage
        self.last_x = 0
        self.last_y = 0

    def initialize(self):
        self.overrun_time = self.timer.get()
        self.last_x = self.controller.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed * self.speed_scalar
        self.last_y = self.controller.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed * self.speed_scalar

    def execute(self) -> None:
        if self.vision.no_sight_range_to_angle(self.drive) != -1:
            # self.set_lookahead_range()
            self.shooter.set_unknown_setpoint(self.vision.no_sight_range_to_angle(self.drive),
                                              VisionConstants.shooter_default_speed)
            if self.shooter.get_ready_to_shoot() and self.vision.get_aligned_odo(self.drive):
                self.vision.align_to_speaker_turret(self.last_x, self.last_y, self.drive)
                self.intake.intake(1)
                self.shooter.shoot()
                self.trapper.advance()
            else:
                self.vision.align_to_speaker_turret(self.controller.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed
                                                    * self.speed_scalar,
                                                    self.controller.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed
                                                    * self.speed_scalar,
                                                    self.drive)
                self.last_x = self.controller.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed * \
                    self.speed_scalar
                self.last_y = self.controller.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed * \
                    self.speed_scalar
        else:
            self.drive.drive_2ok_clt(self.controller.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                                     self.controller.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                                     self.controller.get_axis("RX", 0.06) * -1, self.scale)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        if interrupted:
            self.shooter.set_known_setpoint("readied")
            self.shooter.feeder.set(0)
            self.intake.intake(0)
            self.trapper.manual_trap(0)

    def getInterruptionBehavior(self) -> InterruptionBehavior:
        return InterruptionBehavior.kCancelSelf

    def runsWhenDisabled(self) -> bool:
        return False

    def set_lookahead_range(self):
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            target = Translation2d(VisionConstants.speaker_location_blue[0], VisionConstants.speaker_location_blue[1])
        else:
            target = Translation2d(VisionConstants.speaker_location_red[0], VisionConstants.speaker_location_red[1])

        virtual_goal_x = target.x - self.time_to_shot * (self.drive.vx_new + self.drive.ax * self.accel_comp)
        virtual_goal_y = target.y - self.time_to_shot * (self.drive.vy_new + self.drive.ay * self.accel_comp)

        new_dist = sqrt(pow(self.drive.get_pose().x - virtual_goal_x, 2) +
                        pow(self.drive.get_pose().y - virtual_goal_y, 2))

        # moving_goal_location = Translation2d(virtual_goal_x, virtual_goal_y)

        # to_moving_goal = moving_goal_location - self.drive.get_pose().translation()

        # new_dist = to_moving_goal.distance(self.drive.get_pose().translation())

        # print(new_dist)
        if self.vision.range_to_angle_rand(new_dist) != -1:
            self.shooter.set_unknown_setpoint(self.vision.range_to_angle_rand(new_dist),
                                              VisionConstants.shooter_default_speed)
