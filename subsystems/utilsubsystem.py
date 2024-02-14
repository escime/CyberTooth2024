import commands2
from wpilib import PowerDistribution


class UtilSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

    def toggle_channel(self, on: bool) -> None:
        self.pdh.setSwitchableChannel(on)
