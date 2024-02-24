from commands2 import Command
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from wpilib import DriverStation


class ScoreAMP(Command):

    def __init__(self, trapper: TrapperSubsystem, drive: DriveSubsystem):
        super().__init__()
        self.trapper = trapper
        self.drive = drive
        self.addRequirements(trapper)

    def execute(self):
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            if 0 <= self.drive.get_pose().rotation().degrees() < 180:
                self.trapper.score_in_amp(False)
            else:
                self.trapper.score_in_amp(True)
        else:
            if -180 <= self.drive.get_pose().rotation().degrees() < 0:
                self.trapper.score_in_amp(True)
            else:
                self.trapper.score_in_amp(False)

    def end(self, interrupted: bool):
        self.trapper.stow()
