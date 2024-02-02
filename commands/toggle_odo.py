import commands2
from subsystems.visionsubsystem import VisionSubsystem


class ToggleOdo(commands2.Command):

    def __init__(self, viz: VisionSubsystem):
        super().__init__()
        self.viz = viz
        self.addRequirements(viz)

    def execute(self):
        self.viz.vision_odo_toggle()

    def runsWhenDisabled(self) -> bool:
        return True

    def isFinished(self) -> bool:
        return True
