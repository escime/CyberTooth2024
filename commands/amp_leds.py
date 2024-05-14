from commands2 import Command, InterruptionBehavior
from subsystems.ledsubsystem2 import LEDs


class AmpLEDs(Command):
    def __init__(self, leds: LEDs):
        super().__init__()
        self.leds = leds
        self.addRequirements(leds)

    def initialize(self) -> None:
        self.leds.set_state("timer_lights")

    def isFinished(self) -> bool:
        if self.leds.timer_lights_on is False:
            return True
        else:
            return False

    def getInterruptionBehavior(self) -> InterruptionBehavior:
        return InterruptionBehavior.kCancelIncoming

    def runsWhenDisabled(self) -> bool:
        return True

    def end(self, interrupted: bool):
        self.leds.reset_timer_lights()
        self.leds.set_state("default")
