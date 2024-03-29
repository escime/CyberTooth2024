from commands2 import Command
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem


class Passthrough(Command):
    def __init__(self, fallback: str, shooter: ShooterSubsystem, intake: IntakeSubsystem,
                 trapper: TrapperSubsystem):
        super().__init__()
        self.fallback = fallback
        self.shooter = shooter
        self.intake = intake
        self.trapper = trapper
        self.addRequirements(shooter)
        self.addRequirements(intake)
        self.addRequirements(trapper)

    def initialize(self):
        self.shooter.set_known_setpoint("passthrough")
        self.intake.intake(1)
        self.shooter.shoot()
        self.trapper.manual_trap(1)

    def execute(self):
        self.shooter.shoot()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.shooter.set_known_setpoint(self.fallback)
        self.shooter.feeder.set(0)
        self.intake.intake(0)
        self.trapper.manual_trap(0)
