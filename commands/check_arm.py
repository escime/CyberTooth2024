from commands2 import Command
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from wpilib import Timer, SmartDashboard


class CheckArm(Command):
    def __init__(self, trapper: TrapperSubsystem, util: UtilSubsystem, timer: Timer):
        super().__init__()
        self.trapper = trapper
        self.util = util
        self.timer = timer
        self.addRequirements(trapper)
        self.start_time = self.timer.get()
        self.warning_flags = {"arm": False,
                              "trapper": False}

    def initialize(self):
        self.start_time = self.timer.get()

    def execute(self):
        self.trapper.set_arm("trap")
        self.trapper.trap.set(1)

        if self.check_time(self.util.max_time_arm) and not self.trapper.check_arm_position():
            self.warning_flags["arm"] = True

        SmartDashboard.putNumber("CD Trapper", self.trapper.trap.getOutputCurrent())

        if self.trapper.trap.getOutputCurrent() < self.util.current_warning_low_trapper or \
                self.trapper.trap.getOutputCurrent() > self.util.current_warning_high_trapper:
            self.warning_flags["trapper"] = True

        if any(self.warning_flags):
            SmartDashboard.putBoolean("Master Caution", True)
        else:
            SmartDashboard.putBoolean("Master Caution", False)

        SmartDashboard.putBoolean("Arm Warning", self.warning_flags["arm"])
        SmartDashboard.putBoolean("Trapper Warning", self.warning_flags["trapper"])

    def isFinished(self) -> bool:
        return self.check_time(5)

    def end(self, interrupted: bool):
        self.trapper.stow()

    def check_time(self, time: float) -> bool:
        if self.timer.get() - time > self.start_time:
            return True
        else:
            return False
