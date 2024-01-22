import commands2
from wpilib import PowerDistribution


class UtilSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

        self.motor_array = {19: "FL Drive", 18: "FL Turn", 0: "FR Drive", 1: "FR Turn",
                            10: "BL Drive", 11: "BL Turn", 9: "BR Drive", 8: "BR Turn",
                            3: "Intake Rotation", 2: "Intake", 5: "Stick"}

    def toggle_channel(self, on: bool) -> None:
        self.pdh.setSwitchableChannel(on)
