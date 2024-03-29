from commands2 import Command
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from wpilib import DriverStation


class ShootVisionFeed(Command):
    def __init__(self, shooter: ShooterSubsystem, drive: DriveSubsystem, intake: IntakeSubsystem,
                 trapper: TrapperSubsystem):
        super().__init__()
        self.shooter = shooter
        self.drive = drive
        self.intake = intake
        self.trapper = trapper
        self.addRequirements(shooter)
        self.addRequirements(drive)
        self.addRequirements(intake)
        self.addRequirements(trapper)
        self.target_locked = False
        self.ready_buffer = [False] * 15

    def initialize(self):
        self.target_locked = False

    def execute(self) -> None:
        self.shooter.set_known_setpoint("feed")
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.drive.snap_drive(0, 0, 210)
        else:
            self.drive.snap_drive(0, 0, 140)
        if self.shooter.get_ready_to_shoot():
            self.ready_buffer[0] = True
        else:
            self.ready_buffer[0] = False
        self.ready_buffer = self.ready_buffer[1:] + self.ready_buffer[:1]
        if all(self.ready_buffer):
            self.target_locked = True

    def isFinished(self) -> bool:
        return self.target_locked

    def end(self, interrupted: bool):
        if interrupted:
            self.shooter.set_known_setpoint("readied")
        self.drive.drive_2ok(0, 0, 0, False)
