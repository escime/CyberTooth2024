import commands2
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.shootersubsystem import ShooterSubsystem


class ReadyAMP(commands2.Command):

    def __init__(self, trapper: TrapperSubsystem, shooter: ShooterSubsystem):
        super().__init__()
        self.trapper = trapper
        self.shooter = shooter
        self.addRequirements(trapper)
        self.addRequirements(shooter)

    def initialize(self):
        self.shooter.set_known_setpoint("stow")

    def execute(self):
        if self.shooter.get_ready_to_shoot():
            self.trapper.set_arm("amp")

    def isFinished(self) -> bool:
        if self.trapper.arm_setpoint == "amp":
            return True
        else:
            return False
