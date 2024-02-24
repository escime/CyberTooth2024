from commands2 import Command
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.ledsubsystem import LEDs


class FlashLL(Command):
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
        if self.viz.timer.get() - 1 > self.start_time:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.viz.flash_leds(False)

    def runsWhenDisabled(self) -> bool:
        return True
