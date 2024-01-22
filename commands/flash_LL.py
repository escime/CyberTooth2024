import commands2
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.ledsubsystem import LEDs


class FlashLL(commands2.Command):
    def __init__(self, viz: VisionSubsystem, leds: LEDs):
        super().__init__()
        self.viz = viz
        self.leds = leds
        self.start_time = self.viz.timer.get()
        self.addRequirements(viz)
        self.addRequirements(leds)

    def initialize(self) -> None:
        self.start_time = self.viz.timer.get()

    def execute(self) -> None:
        self.viz.flash_leds(True)
        self.leds.flash_color([255, 0, 0], 2)

    def isFinished(self) -> bool:
        if self.viz.timer.get() - 2 > self.start_time:
            self.viz.flash_leds(False)
            return True
        else:
            return False

    def runsWhenDisabled(self) -> bool:
        return False
