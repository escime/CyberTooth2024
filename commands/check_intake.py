from commands2 import Command
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from wpilib import Timer, SmartDashboard


class CheckIntake(Command):
    def __init__(self, intake: IntakeSubsystem, util: UtilSubsystem, timer: Timer):
        super().__init__()
        self.intake = intake
        self.util = util
        self.timer = timer
        self.addRequirements(intake)
        self.start_time = self.timer.get()
        self.warning_flags = {"intake_1": False,
                              "intake_2": False}

    def initialize(self):
        self.start_time = self.timer.get()

    def execute(self):
        self.intake.intake(1)

        if self.intake.motor.getOutputCurrent() < self.util.current_warning_low_intake or \
                self.intake.motor.getOutputCurrent() > self.util.current_warning_high_intake:
            self.warning_flags["intake_1"] = True

        if self.intake.follower.getOutputCurrent() < self.util.current_warning_low_intake or \
                self.intake.follower.getOutputCurrent() > self.util.current_warning_high_intake:
            self.warning_flags["intake_2"] = True

        if any(self.warning_flags):
            SmartDashboard.putBoolean("Master Caution", True)
        else:
            SmartDashboard.putBoolean("Master Caution", False)

        SmartDashboard.putBoolean("Intake 1 Warning", self.warning_flags["intake_1"])
        SmartDashboard.putBoolean("Intake 2 Warning", self.warning_flags["intake_2"])

    def isFinished(self) -> bool:
        return self.check_time(3)

    def end(self, interrupted: bool):
        self.intake.intake(0)

    def check_time(self, time: float) -> bool:
        if self.timer.get() - time > self.start_time:
            return True
        else:
            return False
