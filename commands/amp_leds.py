import commands2
from subsystems.ledsubsystem import LEDs


class AmpLEDs(commands2.CommandBase):
    def __init__(self, leds: LEDs):
        super().__init__()
        self.leds = leds
        self.addRequirements(leds)

    def initialize(self) -> None:
        self.leds.amp_timer()

    def execute(self) -> None:
        self.leds.amp_timer()

    def isFinished(self) -> bool:
        if self.leds.amp_timer_on is False:
            return True
        else:
            return False

    def runsWhenDisabled(self) -> bool:
        return False
