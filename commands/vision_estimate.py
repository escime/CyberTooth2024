from commands2 import Command
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.visionsubsystem import VisionSubsystem


class VisionEstimate(Command):
    def __init__(self, vision: VisionSubsystem, drive: DriveSubsystem):
        super().__init__()
        self.vision = vision
        self.drive = drive
        self.addRequirements(vision, drive)

    def initialize(self) -> None:
        self.vision.conditional_instant_update(self.drive)

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return True
