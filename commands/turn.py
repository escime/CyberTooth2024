import commands2
from subsystems.drivesubsystem import DriveSubsystem


class Turn(commands2.CommandBase):
    def __init__(self, robot_drive: DriveSubsystem, target):
        super().__init__()
        self.robot_drive = robot_drive
        self.target = target
        self.addRequirements(robot_drive)

    def execute(self) -> None:
        self.robot_drive.snap_drive(0, 0, self.target)

    def isFinished(self) -> bool:
        if self.target - 5 < abs(self.robot_drive.get_heading()) < self.target + 5:
            return True
        else:
            return False
