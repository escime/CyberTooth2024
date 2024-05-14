from commands2 import Command
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from wpilib import Timer, SmartDashboard


class CheckShooter(Command):
    def __init__(self, shooter: ShooterSubsystem, util: UtilSubsystem, timer: Timer):
        super().__init__()
        self.shooter = shooter
        self.util = util
        self.timer = timer
        self.addRequirements(shooter)
        self.start_time = self.timer.get()
        self.warning_flags = {"shooter_top": False,
                              "shooter_bottom": False,
                              "angler": False,
                              "feeder": False,
                              "spinup": False}
        self.lockout1 = True
        self.lockout2 = True

    def initialize(self):
        self.start_time = self.timer.get()
        self.lockout1 = True
        self.lockout2 = True

    def execute(self):
        self.shooter.set_known_setpoint("podium")
        if self.shooter.get_at_angle() and self.lockout1:
            if self.check_time(self.util.max_time_angler):
                self.warning_flags["angler"] = True
            self.lockout1 = False
        if self.shooter.get_at_speed() and self.lockout2:
            if self.check_time(self.util.max_time_shooter):
                self.warning_flags["spinup"] = True
            self.lockout2 = False

        if self.shooter.get_ready_to_shoot() and self.check_time(3):
            self.shooter.shoot()
            current_draw_top = self.shooter.shooter_top.getOutputCurrent()
            current_draw_bottom = self.shooter.shooter_bottom.getOutputCurrent()
            current_draw_feeder = self.shooter.feeder.getOutputCurrent()

            SmartDashboard.putNumber("CD Shooter Top", current_draw_top)
            SmartDashboard.putNumber("CD Shooter Bottom", current_draw_bottom)
            SmartDashboard.putNumber("CD Feeder", current_draw_feeder)

            if current_draw_top < self.util.current_warning_low_shooter or \
                    current_draw_top > self.util.current_warning_high_shooter:
                self.warning_flags["shooter_top"] = True
            if current_draw_bottom < self.util.current_warning_low_shooter or \
                    current_draw_bottom > self.util.current_warning_high_shooter:
                self.warning_flags["shooter_bottom"] = True
            if current_draw_feeder < self.util.current_warning_low_feeder or \
                    current_draw_feeder > self.util.current_warning_high_feeder:
                self.warning_flags["feeder"] = True
        elif self.check_time(3) and not self.shooter.get_ready_to_shoot():
            self.warning_flags["spinup"] = True

        if any(self.warning_flags):
            SmartDashboard.putBoolean("Master Caution", True)
        else:
            SmartDashboard.putBoolean("Master Caution", False)

        SmartDashboard.putBoolean("Shooter Top Warning", self.warning_flags["shooter_top"])
        SmartDashboard.putBoolean("Shooter Bottom Warning", self.warning_flags["shooter_bottom"])
        SmartDashboard.putBoolean("Feeder Warning", self.warning_flags["feeder"])
        SmartDashboard.putBoolean("Angler Warning", self.warning_flags["angler"])
        SmartDashboard.putBoolean("Spinup Warning", self.warning_flags["spinup"])

    def isFinished(self) -> bool:
        return self.check_time(7)

    def end(self, interrupted: bool):
        self.shooter.set_known_setpoint("stow")
        self.shooter.feeder.set(0)

    def check_time(self, time: float) -> bool:
        if self.timer.get() - time > self.start_time:
            return True
        else:
            return False
