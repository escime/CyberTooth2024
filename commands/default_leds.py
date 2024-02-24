from commands2 import Command
from subsystems.ledsubsystem import LEDs


class DefaultLEDs(Command):
    def __init__(self, leds: LEDs):
        super().__init__()
        self.leds = leds
        self.addRequirements(leds)

    def initialize(self) -> None:
        self.leds.purple_chaser()

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return True
