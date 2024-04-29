from commands2 import Subsystem
from phoenix6.hardware import CANcoder


class EncoderSubsystem(Subsystem):
    """A subsystem for controlling motors on the CUBE."""
    def __init__(self):
        super().__init__()

        self.encoder = CANcoder(32, "rio")

    def get_absolute_position(self) -> float:
        return self.encoder.get_absolute_position().value_as_double

    def get_relative_position(self) -> float:
        return self.encoder.get_position().value_as_double

    def set_relative_position(self, pos: float) -> None:
        self.encoder.set_position(pos)
