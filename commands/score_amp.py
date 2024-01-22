import commands2
from subsystems.trappersubsystem import TrapperSubsystem


class ScoreAMP(commands2.Command):

    def __init__(self, trapper: TrapperSubsystem):
        super().__init__()
        self.trapper = trapper
        self.addRequirements(trapper)

    def execute(self):
        self.trapper.score_in_amp()

    def end(self, interrupted: bool):
        self.trapper.stow()
