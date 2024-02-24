from commands2 import Command
from subsystems.shootersubsystem import ShooterSubsystem
from wpilib import Timer


class ReadyShooter(Command):
    def __init__(self, shooter: ShooterSubsystem, setpoint: str, timer: Timer):
        super().__init__()
        self.shooter = shooter
        self.setpoint = setpoint
        self.addRequirements(shooter)
        self.timer = timer
        self.start_time = 0

    def initialize(self) -> None:
        self.shooter.set_known_setpoint(self.setpoint)
        self.start_time = self.timer.get()

    def isFinished(self) -> bool:
        if self.shooter.get_ready_to_shoot() or self.timer.get() - 3 > self.start_time:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        print("ReadyShooter complete.")
