import commands2
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from wpilib import Timer


class Shoot(commands2.Command):
    def __init__(self, fallback: str, bypass_timeout: bool, shooter: ShooterSubsystem, intake: IntakeSubsystem,
                 trapper: TrapperSubsystem):
        super().__init__()
        self.fallback = fallback
        self.bypass_timeout = bypass_timeout
        self.shooter = shooter
        self.intake = intake
        self.trapper = trapper
        self.addRequirements(shooter)
        self.addRequirements(intake)
        self.addRequirements(trapper)
        self.timer = Timer()
        self.timer_2 = Timer()
        self.start_time = 0
        self.start_time_2 = 1000

    def initialize(self):
        self.timer.start()
        self.timer_2.start()
        self.start_time = self.timer.get()
        self.intake.intake(1)
        self.shooter.shoot()
        self.trapper.advance()

    def execute(self):
        if not self.trapper.get_note_acquired():
            self.start_time_2 = self.timer_2.get()

    def isFinished(self) -> bool:
        if not self.bypass_timeout:
            if self.timer.get() - 5 > self.start_time or self.timer_2.get() - 0.5 > self.start_time_2:
                return True
            else:
                return False
        else:
            return False

    def end(self, interrupted: bool):
        self.shooter.set_known_setpoint(self.fallback)
        self.shooter.feeder.set(0)
        self.intake.intake(0)
        self.trapper.manual_trap(0)
        print("Shoot complete.")
