from commands2 import Command
from subsystems.drivesubsystem import DriveSubsystem


class DebugMode(Command):
    def __init__(self, robot_drive: DriveSubsystem, on: bool):
        super().__init__()
        self.robot_drive = robot_drive
        self.addRequirements(robot_drive)
        self.intent = on

    def execute(self) -> None:
        self.robot_drive.debug_toggle(self.intent)

    def runsWhenDisabled(self) -> bool:
        return True
