import commands2
from wpilib import PowerDistribution


class UtilSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        self.master_caution = False
        self.drivetrain_caution = False
        self.intake_caution = False
        self.abnormal_current_alarm = False
        self.channel_overcurrent = [False, False, False, False, False,
                                    False, False, False, False, False,
                                    False, False, False, False, False,
                                    False, False, False, False, False,
                                    False, False, False, False]
        self.normal_current = [30, 30, 30, 30, 30,
                               30, 30, 30, 30, 30,
                               30, 30, 30, 30, 30,
                               30, 30, 30, 30, 30,
                               15, 15, 15, 15]
        self.motor_array = {19: "FL Drive", 18: "FL Turn", 0: "FR Drive", 1: "FR Turn",
                            10: "BL Drive", 11: "BL Turn", 9: "BR Drive", 8: "BR Turn",
                            3: "Intake Rotation", 2: "Intake", 5: "Stick"}
        self.overcurrented_motors = ""

    def record_error(self) -> None:
        for i in range(0, 23):
            if self.pdh.getCurrent(i) > self.normal_current[i]:
                self.channel_overcurrent[i] = True
                self.abnormal_current_alarm = True
        for i in range(0, 23):
            if self.channel_overcurrent[i] is True:
                self.overcurrented_motors = self.overcurrented_motors + self.motor_array[i] + ","

    def toggle_channel(self, on: bool) -> None:
        self.pdh.setSwitchableChannel(on)
