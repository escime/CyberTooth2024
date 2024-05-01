from commands2 import Command
from subsystems.phoenixdrivesubsystem import PhoenixDriveSubsystem


class DebugMode(Command):
    def __init__(self, drive: PhoenixDriveSubsystem, on: bool):
        super().__init__()
        self.drive = drive
        self.addRequirements(drive)
        self.intent = on

    def execute(self) -> None:
        self.drive.debug_switch(self.intent)

    def runsWhenDisabled(self) -> bool:
        return True
