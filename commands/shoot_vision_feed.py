from commands2 import Command
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from helpers.custom_hid import CustomHID
from wpilib import DriverStation
from constants import DriveConstants


class ShootVisionFeed(Command):
    def __init__(self, shooter: ShooterSubsystem, drive: DriveSubsystem, intake: IntakeSubsystem,
                 trapper: TrapperSubsystem, vision: VisionSubsystem, controller: CustomHID):
        super().__init__()
        self.shooter = shooter
        self.drive = drive
        self.intake = intake
        self.trapper = trapper
        self.vision = vision
        self.controller = controller
        self.addRequirements(shooter)
        self.addRequirements(drive)
        self.addRequirements(intake)
        self.addRequirements(trapper)
        self.target_locked = False
        self.ready_buffer = [False] * 6

    def initialize(self):
        self.target_locked = False

    def execute(self) -> None:
        # self.shooter.set_known_setpoint("feed")
        # if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        #     self.drive.snap_drive(0, 0, 210)
        # else:
        #     self.drive.snap_drive(0, 0, 140)
        # if self.shooter.get_ready_to_shoot():
        #     self.ready_buffer[0] = True
        # else:
        #     self.ready_buffer[0] = False
        if self.vision.range_to_feed(self.drive) != -1:
            if self.vision.range_to_speaker_odo(self.drive) > 5.5:
                self.shooter.set_unknown_setpoint(self.vision.range_to_feed(self.drive),
                                                  3750)
            else:
                self.shooter.set_unknown_setpoint(self.vision.range_to_feed(self.drive),
                                                  3250)
        self.vision.align_to_speaker_odo(self.controller.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed * 0.9,
                                         self.controller.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed * 0.9,
                                         self.drive)
        if self.shooter.get_ready_to_shoot() and self.vision.get_aligned_odo(5, self.drive):
            self.ready_buffer[0] = True
        else:
            self.ready_buffer[0] = False
        self.ready_buffer = self.ready_buffer[1:] + self.ready_buffer[:1]
        if all(self.ready_buffer):
            self.intake.intake(1)
            self.shooter.shoot()
            self.trapper.advance()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        if interrupted:
            self.shooter.set_known_setpoint("readied")
            self.shooter.feeder.set(0)
            self.intake.intake(0)
            self.trapper.manual_trap(0)
