import commands2
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from wpilib import Timer


class Shoot(commands2.Command):
    def __init__(self, shooter: ShooterSubsystem, intake: IntakeSubsystem, trapper: TrapperSubsystem):
        super().__init__()
        self.shooter = shooter
        self.intake = intake
        self.trapper = trapper
        self.addRequirements(shooter)
        self.addRequirements(intake)
        self.addRequirements(trapper)
        self.timer = Timer()
        self.start_time = 0

    def initialize(self):
        self.timer.start()
        self.start_time = self.timer.get()
        self.intake.intake(1)
        self.shooter.shoot()
        self.trapper.advance()

    def isFinished(self) -> bool:
        # TODO replace with a different condition later.
        if self.timer.get() - 1 > self.start_time:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.shooter.set_known_setpoint("readied")
        self.intake.intake(0)
        self.trapper.manual_trap(0)
        print("Shoot complete.")
