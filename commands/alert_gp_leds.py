from commands2 import Command
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.ledsubsystem import LEDs


class AlertGPLEDs(Command):
    def __init__(self, leds: LEDs, trapper: TrapperSubsystem):
        super().__init__()
        self.leds = leds
        self.trapper = trapper
        self.addRequirements(leds)

    def execute(self) -> None:
        self.leds.green_chaser()

    def isFinished(self) -> bool:
        if not self.trapper.get_note_acquired():
            return True
        else:
            return False

    def runsWhenDisabled(self) -> bool:
        return True
