from commands2 import Command
from subsystems.ledsubsystem2 import LEDs
from wpilib import Timer, SmartDashboard


class MasterCaution(Command):
    def __init__(self, leds: LEDs, timer: Timer):
        super().__init__()
        self.leds = leds
        self.timer = timer
        self.addRequirements(leds)
        self.start_time = self.timer.get()

    def initialize(self):
        self.start_time = self.timer.get()
        if SmartDashboard.getBoolean("Master Caution", False):
            self.leds.set_flash_color_color([0, 255, 0])
        else:
            self.leds.set_flash_color_color([255, 0, 0])
        self.leds.set_state("flash_color")

    def isFinished(self) -> bool:
        return self.check_time(20)

    def check_time(self, time: float) -> bool:
        if self.timer.get() - time > self.start_time:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.leds.set_state("default")
