from commands2 import Command
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from constants import DriveConstants
from wpilib import Timer, SmartDashboard


class CheckDrive(Command):
    def __init__(self, drive: DriveSubsystem, util: UtilSubsystem, timer: Timer):
        super().__init__()
        self.drive = drive
        self.util = util
        self.timer = timer
        self.addRequirements(drive)
        self.start_time = self.timer.get()
        self.warning_flags = [[False, False], [False, False], [False, False], [False, False]]

    def initialize(self):
        self.start_time = self.timer.get()

    def execute(self):
        if not self.check_time(3):
            self.drive.drive(DriveConstants.kMaxSpeed, 0, 0, False)
            if not self.check_time(2):
                current_draw = self.drive.get_current_draw_all_modules()
                SmartDashboard.putNumber("FL Drive CD", current_draw[0][0])
                SmartDashboard.putNumber("FL Steer CD", current_draw[0][1])
                SmartDashboard.putNumber("FR Drive CD", current_draw[1][0])
                SmartDashboard.putNumber("FR Steer CD", current_draw[1][1])
                SmartDashboard.putNumber("BL Drive CD", current_draw[2][0])
                SmartDashboard.putNumber("BL Steer CD", current_draw[2][1])
                SmartDashboard.putNumber("BR Drive CD", current_draw[3][0])
                SmartDashboard.putNumber("BR Steer CD", current_draw[3][1])
                for i in range(0, 4):
                    if current_draw[i][0] < self.util.current_warning_low_drive_motor or \
                            current_draw[i][0] > self.util.current_warning_high_drive_motor:
                        self.warning_flags[i][0] = True
                    if current_draw[i][1] < self.util.current_warning_low_steer_motor or \
                            current_draw[i][1] > self.util.current_warning_high_steer_motor:
                        self.warning_flags[i][1] = True
        elif not self.check_time(6) and self.check_time(3):
            self.drive.drive(0, DriveConstants.kMaxSpeed, 0, False)
            if self.check_time(5):
                current_draw = self.drive.get_current_draw_all_modules()
                SmartDashboard.putNumber("FL Drive CD", current_draw[0][0])
                SmartDashboard.putNumber("FL Steer CD", current_draw[0][1])
                SmartDashboard.putNumber("FR Drive CD", current_draw[1][0])
                SmartDashboard.putNumber("FR Steer CD", current_draw[1][1])
                SmartDashboard.putNumber("BL Drive CD", current_draw[2][0])
                SmartDashboard.putNumber("BL Steer CD", current_draw[2][1])
                SmartDashboard.putNumber("BR Drive CD", current_draw[3][0])
                SmartDashboard.putNumber("BR Steer CD", current_draw[3][1])
                for i in range(0, 4):
                    if current_draw[i][0] < self.util.current_warning_low_drive_motor or \
                            current_draw[i][0] > self.util.current_warning_high_drive_motor:
                        self.warning_flags[i][0] = True
                    if current_draw[i][1] < self.util.current_warning_low_steer_motor or \
                            current_draw[i][1] > self.util.current_warning_high_steer_motor:
                        self.warning_flags[i][1] = True
        elif not self.check_time(9) and self.check_time(6):
            self.drive.drive(0, 0, DriveConstants.kMaxAngularSpeed, False)
            if self.check_time(8):
                current_draw = self.drive.get_current_draw_all_modules()
                SmartDashboard.putNumber("FL Drive CD", current_draw[0][0])
                SmartDashboard.putNumber("FL Steer CD", current_draw[0][1])
                SmartDashboard.putNumber("FR Drive CD", current_draw[1][0])
                SmartDashboard.putNumber("FR Steer CD", current_draw[1][1])
                SmartDashboard.putNumber("BL Drive CD", current_draw[2][0])
                SmartDashboard.putNumber("BL Steer CD", current_draw[2][1])
                SmartDashboard.putNumber("BR Drive CD", current_draw[3][0])
                SmartDashboard.putNumber("BR Steer CD", current_draw[3][1])
                for i in range(0, 4):
                    if current_draw[i][0] < self.util.current_warning_low_drive_motor or \
                            current_draw[i][0] > self.util.current_warning_high_drive_motor:
                        self.warning_flags[i][0] = True
                    if current_draw[i][1] < self.util.current_warning_low_steer_motor or \
                            current_draw[i][1] > self.util.current_warning_high_steer_motor:
                        self.warning_flags[i][1] = True
        if any(self.warning_flags[0]) or any(self.warning_flags[1]) or \
                any(self.warning_flags[2]) or any(self.warning_flags[3]):
            SmartDashboard.putBoolean("Master Caution", True)
        else:
            SmartDashboard.putBoolean("Master Caution", False)
        SmartDashboard.putBoolean("FL Drive Warning", self.warning_flags[0][0])
        SmartDashboard.putBoolean("FL Steer Warning", self.warning_flags[0][1])
        SmartDashboard.putBoolean("FR Drive Warning", self.warning_flags[1][0])
        SmartDashboard.putBoolean("FR Steer Warning", self.warning_flags[1][1])
        SmartDashboard.putBoolean("BL Drive Warning", self.warning_flags[2][0])
        SmartDashboard.putBoolean("BL Steer Warning", self.warning_flags[2][1])
        SmartDashboard.putBoolean("BR Drive Warning", self.warning_flags[3][0])
        SmartDashboard.putBoolean("BR Steer Warning", self.warning_flags[3][1])

    def isFinished(self) -> bool:
        return self.check_time(9)

    def end(self, interrupted: bool):
        self.drive.drive(0, 0, 0, False)

    def check_time(self, time: float) -> bool:
        if self.timer.get() - time > self.start_time:
            return True
        else:
            return False
