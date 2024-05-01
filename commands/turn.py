from commands2 import Command
from subsystems.phoenixdrivesubsystem import PhoenixDriveSubsystem
from wpilib import Timer


class Turn(Command):
    def __init__(self, drive: PhoenixDriveSubsystem, target, timer: Timer):
        super().__init__()
        self.drive = drive
        self.target = target
        self.addRequirements(drive)
        self.timer = timer
        self.start_time = 0

    def initialize(self):
        self.timer.start()
        self.start_time = self.timer.get()

    def execute(self) -> None:
        self.drive.snap_drive(0, 0, self.target)

    def isFinished(self) -> bool:
        if self.target - 5 < abs(self.drive.get_heading()) < self.target + 5 or \
                self.timer.get() - 3 > self.start_time:
            return True
        else:
            return False
