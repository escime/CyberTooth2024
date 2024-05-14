import commands2
from wpilib import PowerDistribution


class UtilSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

        self.current_warning_high_drive_motor = 50
        self.current_warning_low_drive_motor = 10
        self.current_warning_high_steer_motor = 30
        self.current_warning_low_steer_motor = 10

        self.current_warning_high_shooter = 50
        self.current_warning_low_shooter = 10
        self.current_warning_high_feeder = 40
        self.current_warning_low_feeder = 10
        self.max_time_angler = 1
        self.max_time_shooter = 2

        self.max_time_arm = 4
        self.current_warning_high_trapper = 50
        self.current_warning_low_trapper = 10

        self.current_warning_high_intake = 50
        self.current_warning_low_intake = 10

    def toggle_channel(self, on: bool) -> None:
        self.pdh.setSwitchableChannel(on)
