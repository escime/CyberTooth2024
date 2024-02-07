import commands2
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.shootersubsystem import ShooterSubsystem


class ReadyAMPNoShooter(commands2.Command):

    def __init__(self, trapper: TrapperSubsystem):
        super().__init__()
        self.trapper = trapper
        self.addRequirements(trapper)

    def execute(self):
        self.trapper.set_arm("amp")

    def isFinished(self) -> bool:
        if self.trapper.arm_setpoint == "amp":
            return True
        else:
            return False
