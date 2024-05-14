from commands2 import Command
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.ledsubsystem2 import LEDs


class AlertGPLEDs(Command):
    def __init__(self, leds: LEDs, trapper: TrapperSubsystem):
        super().__init__()
        self.leds = leds
        self.trapper = trapper
        self.addRequirements(leds)

    def initialize(self):
        self.leds.set_state("gp_held")

    def isFinished(self) -> bool:
        if not self.trapper.get_note_acquired():
            return True
        else:
            return False

    def runsWhenDisabled(self) -> bool:
        return True

    def end(self, interrupted: bool):
        self.leds.set_state("default")
