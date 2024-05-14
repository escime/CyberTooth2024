from commands2 import Command
from subsystems.drivesubsystem import DriveSubsystem
from wpilib import Timer


class ReturnWheels(Command):
    def __init__(self, robot_drive: DriveSubsystem):
        super().__init__()
        self.robot_drive = robot_drive
        self.addRequirements(robot_drive)
        self.timer = Timer()
        self.start_time = 0

    def initialize(self):
        self.timer.start()
        self.start_time = self.timer.get()

    def execute(self) -> None:
        self.robot_drive.return_wheels_to_zero()

    def isFinished(self) -> bool:
        angle = self.robot_drive.m_FL.get_state_onboard().angle.degrees()
        if 0 < abs(angle) < 10 or 170 < abs(angle) < 190 or 350 < abs(angle) < 360:
            return True
        elif self.timer.get() - 2 > self.start_time:
            return True
        else:
            return False
