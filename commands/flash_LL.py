from commands2 import Command
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.ledsubsystem2 import LEDs
from wpilib import Timer


class FlashLL(Command):
    def __init__(self, viz: VisionSubsystem, leds: LEDs, timer: Timer):
        super().__init__()
        self.viz = viz
        self.leds = leds
        self.timer = timer
        self.start_time = self.timer.get()
        self.addRequirements(viz)
        self.addRequirements(leds)

    def initialize(self) -> None:
        self.start_time = self.timer.get()
        self.leds.set_flash_color_color([255, 0, 0])
        self.leds.set_flash_color_rate(5)

    def execute(self) -> None:
        self.viz.flash_leds(True)
        self.leds.set_state("flash_color")

    def isFinished(self) -> bool:
        if self.timer.get() - 2 > self.start_time:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.viz.flash_leds(False)
        self.leds.set_state("gp_held")

    def runsWhenDisabled(self) -> bool:
        return True
