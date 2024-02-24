from commands2 import Command, InterruptionBehavior
from subsystems.ledsubsystem import LEDs


class AmpLEDs(Command):
    def __init__(self, leds: LEDs):
        super().__init__()
        self.leds = leds
        self.addRequirements(leds)

    def execute(self) -> None:
        self.leds.amp_timer()

    def isFinished(self) -> bool:
        if self.leds.amp_timer_on is False:
            return True
        else:
            return False

    def getInterruptionBehavior(self) -> InterruptionBehavior:
        return InterruptionBehavior.kCancelIncoming

    def runsWhenDisabled(self) -> bool:
        return True
