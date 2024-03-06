from commands2 import Command, InterruptionBehavior
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from constants import VisionConstants


class MaintainShooter(Command):
    def __init__(self, shooter: ShooterSubsystem, drive: DriveSubsystem, viz: VisionSubsystem):
        super().__init__()
        self.shooter = shooter
        self.drive = drive
        self.viz = viz
        self.addRequirements(shooter)

    def execute(self) -> None:
        if self.viz.range_to_angle_m(self.drive) != -1:
            self.shooter.set_unknown_setpoint(self.viz.range_to_angle_m(self.drive),
                                              VisionConstants.shooter_default_speed)
        else:
            self.shooter.set_known_setpoint("readied")

    def isFinished(self) -> bool:
        return False

    def getInterruptionBehavior(self) -> InterruptionBehavior:
        return InterruptionBehavior.kCancelSelf
