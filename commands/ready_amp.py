from commands2 import Command
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from wpilib import DriverStation


class ReadyAMP(Command):

    def __init__(self, trapper: TrapperSubsystem, shooter: ShooterSubsystem, drive: DriveSubsystem,
                 intake: IntakeSubsystem):
        super().__init__()
        self.trapper = trapper
        self.shooter = shooter
        self.drive = drive
        self.intake = intake
        self.addRequirements(trapper)
        self.addRequirements(shooter)
        self.addRequirements(intake)
        # Didn't include drive as a requirement since it is pull request only.

    def initialize(self):
        self.shooter.set_known_setpoint("stow")

    def execute(self):
        if self.shooter.get_ready_to_shoot():
            self.intake.intake(0.5)
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                if 0 <= self.drive.get_pose().rotation().degrees() < 180:
                    self.trapper.set_arm("amp")
                else:
                    self.trapper.set_arm("trap")
            else:
                if -180 <= self.drive.get_pose().rotation().degrees() < 0:
                    self.trapper.set_arm("trap")
                else:
                    self.trapper.set_arm("amp")
        else:
            self.shooter.set_known_setpoint("stow")
            self.intake.intake(0.5)

    def isFinished(self) -> bool:
        if self.trapper.check_arm_position() and (self.trapper.arm_setpoint == "amp" or
                                                  self.trapper.arm_setpoint == "trap"):
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.intake.intake(0)
