import commands2
from subsystems.ledsubsystem import LEDs


class ShootLEDs(commands2.CommandBase):
    def __init__(self, leds: LEDs, speed: str):
        super().__init__()
        self.leds = leds
        self.speed = speed
        self.addRequirements(leds)

    def initialize(self) -> None:
        self.leds.shoot_animator(self.speed)

    def execute(self) -> None:
        self.leds.shoot_animator(self.speed)

    def isFinished(self) -> bool:
        if self.leds.shooting is False:
            return True
        else:
            return False

    def runsWhenDisabled(self) -> bool:
        return False
