import commands2
from subsystems.drivesubsystem import DriveSubsystem
from wpilib import Timer


class Turn(commands2.Command):
    def __init__(self, robot_drive: DriveSubsystem, target):
        super().__init__()
        self.robot_drive = robot_drive
        self.target = target
        self.addRequirements(robot_drive)
        self.timer = Timer()
        self.start_time = 0

    def initialize(self):
        self.timer.start()
        self.start_time = self.timer.get()

    def execute(self) -> None:
        self.robot_drive.snap_drive(0, 0, self.target)

    def isFinished(self) -> bool:
        if self.target - 5 < abs(self.robot_drive.get_heading()) < self.target + 5 or \
                self.timer.get() - 3 > self.start_time:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        print("Turn complete.")
