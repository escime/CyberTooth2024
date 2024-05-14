from commands2 import Command
from subsystems.ledsubsystem2 import LEDs


class ShootLEDs(Command):
    def __init__(self, leds: LEDs):
        super().__init__()
        self.leds = leds
        self.addRequirements(leds)

    def initialize(self) -> None:
        self.leds.set_state("shoot")

    def runsWhenDisabled(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.leds.reset_shoot()
        self.leds.set_state("default")
