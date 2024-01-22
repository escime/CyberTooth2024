import commands2
from subsystems.shootersubsystem import ShooterSubsystem


class ReadyShooter(commands2.CommandBase):
    def __init__(self, shooter: ShooterSubsystem, setpoint: str):
        super().__init__()
        self.shooter = shooter
        self.setpoint = setpoint
        self.addRequirements(shooter)

    def initialize(self) -> None:
        self.shooter.set_known_setpoint(self.setpoint)

    def isFinished(self) -> bool:
        return self.shooter.get_ready_to_shoot()
