import commands2
from subsystems.visionsubsystem import VisionSubsystem


class FlashLL(commands2.CommandBase):
    def __init__(self, viz: VisionSubsystem):
        super().__init__()
        self.viz = viz
        self.start_time = self.viz.timer.get()
        self.addRequirements(viz)

    def initialize(self) -> None:
        self.start_time = self.viz.timer.get()

    def execute(self) -> None:
        self.viz.flash_leds(True)

    def isFinished(self) -> bool:
        if self.viz.timer.get() - 1 > self.start_time:
            self.viz.flash_leds(False)
            return True
        else:
            return False

    def runsWhenDisabled(self) -> bool:
        return False
