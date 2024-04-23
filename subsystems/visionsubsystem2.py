from helpers import limelight, limelightresults
from commands2 import Subsystem
from wpilib import Timer
from subsystems.drivesubsystem import DriveSubsystem


class VisionSubsystem2(Subsystem):

    use_megatag2 = True

    def __init__(self, timer: Timer, drive: DriveSubsystem):
        super().__init__()
        self.timer = timer
        self.drive = drive
        limelight_addresses = ["10.39.40.11"]
        limelights = []
        for x in limelight_addresses:
            limelights.append(limelight.Limelight(x))

    def periodic(self) -> None:
        if self.use_megatag2:
            self.set_robot_orientation()

    def set_robot_orientation(self, limelight_id: int):
        entries = [self.drive.gyro.getYaw(), 0, 0, 0, 0, 0]

