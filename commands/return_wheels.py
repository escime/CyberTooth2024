import commands2
from subsystems.drivesubsystem import DriveSubsystem


class ReturnWheels(commands2.CommandBase):
    def __init__(self, robot_drive: DriveSubsystem):
        super().__init__()
        self.robot_drive = robot_drive
        self.addRequirements(robot_drive)

    def execute(self) -> None:
        self.robot_drive.return_wheels_to_zero()

    def isFinished(self) -> bool:
        angle = self.robot_drive.m_FL.get_state().angle.degrees()
        if 0 < abs(angle) < 10 or 170 < abs(angle) < 190 or 350 < abs(angle) < 360:
            return True
        else:
            return False
