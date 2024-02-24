from commands2 import Command
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.shootersubsystem import ShooterSubsystem


class Eject(Command):

    def __init__(self, intake: IntakeSubsystem, trapper: TrapperSubsystem, shooter: ShooterSubsystem):
        super().__init__()
        self.intake = intake
        self.trapper = trapper
        self.shooter = shooter
        self.addRequirements(intake)
        self.addRequirements(trapper)
        self.addRequirements(shooter)

    def execute(self):
        self.intake.intake(-1)
        self.trapper.manual_trap(-1)
        self.shooter.eject()

    def end(self, interrupted: bool):
        self.intake.intake(0)
        self.trapper.manual_trap(0)
        self.shooter.zero_out()
