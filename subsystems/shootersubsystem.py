import commands2
from rev import CANSparkMax, SparkMaxAbsoluteEncoder
from constants import ShooterConstants
from wpilib import SmartDashboard


class ShooterSubsystem(commands2.Subsystem):

    shooter_setpoints = {"stow": 0, "subwoofer": 5300, "podium": 5300, "readied": 2000, "test": 5300, "feed": 3750,
                         "passthrough": 2000}
    # angle_setpoints = {"stow": 0.867, "subwoofer": 0.682, "podium": 0.622, "readied": 0.7, "test": 0.736}
    angle_setpoints = {"stow": 0.867, "subwoofer": 0.705, "podium": 0.75, "readied": 0.79, "test": 0.765, "feed": 0.73,
                       "passthrough": 0.765}
    # previous subwoofer 0.710

    def __init__(self) -> None:
        super().__init__()
        self.shooter_top = CANSparkMax(ShooterConstants.top_id, CANSparkMax.MotorType.kBrushless)
        self.shooter_bottom = CANSparkMax(ShooterConstants.bottom_id, CANSparkMax.MotorType.kBrushless)
        self.shooter_top.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.shooter_bottom.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.shooter_top.setSmartCurrentLimit(ShooterConstants.current_limit)
        self.shooter_bottom.setSmartCurrentLimit(ShooterConstants.current_limit)
        self.shooter_pid_top = self.shooter_top.getPIDController()
        self.shooter_pid_bottom = self.shooter_bottom.getPIDController()
        self.shooter_pid_top.setFF(ShooterConstants.shooter_kFF)
        self.shooter_pid_top.setP(ShooterConstants.shooter_kP)
        self.shooter_pid_top.setD(ShooterConstants.shooter_kD)
        self.shooter_pid_bottom.setFF(ShooterConstants.shooter_kFF)
        self.shooter_pid_bottom.setP(ShooterConstants.shooter_kP)
        self.shooter_pid_bottom.setD(ShooterConstants.shooter_kD)
        self.shooter_encoder_top = self.shooter_top.getEncoder()
        self.shooter_encoder_bottom = self.shooter_bottom.getEncoder()

        self.angler = CANSparkMax(38, CANSparkMax.MotorType.kBrushless)
        self.encoder = self.angler.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.angler.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.angle_pid = self.angler.getPIDController()
        # self.encoder.setZeroOffset(0.554 + 0.1)
        self.encoder.setZeroOffset(0.593 + 0.1)
        self.angle_pid.setFeedbackDevice(self.encoder)
        self.angle_pid.setP(ShooterConstants.angle_kP)
        self.angle_pid.setI(ShooterConstants.angle_kI)
        self.angle_pid.setOutputRange(-0.8, 0.8)
        self.angle_pid.setPositionPIDWrappingEnabled(True)

        self.feeder = CANSparkMax(32, CANSparkMax.MotorType.kBrushless)
        self.feeder.setIdleMode(CANSparkMax.IdleMode.kBrake)

        self.shooter_top.setControlFramePeriodMs(50)
        self.shooter_bottom.setControlFramePeriodMs(50)
        self.angler.setControlFramePeriodMs(60)
        self.feeder.setControlFramePeriodMs(120)

        self.shooter_top.burnFlash()
        self.shooter_bottom.burnFlash()
        self.angler.burnFlash()
        self.feeder.burnFlash()

        self.shooter_setpoint = 0
        self.angle_setpoint = self.angle_setpoints["stow"]
        self.trim = ShooterConstants.trim

        self.default_bypass = False
        self.current_detection_buffer_top = [False] * 20
        self.current_detection_buffer_bottom = [False] * 20

    def shoot(self) -> None:
        """Advance Note into shooter wheels."""
        self.feeder.set(ShooterConstants.feeder_speed)

    def spin_up(self, speed: float) -> None:
        """Spin up shooter."""
        if speed == 0:
            self.shooter_top.set(0)
            self.shooter_bottom.set(0)
        else:
            self.shooter_pid_top.setReference(speed, CANSparkMax.ControlType.kVelocity)
            self.shooter_pid_bottom.setReference(speed, CANSparkMax.ControlType.kVelocity)
        self.shooter_setpoint = speed

    def increment_trim(self, amount: float):
        self.trim += amount

    def get_at_speed(self) -> bool:
        """Check up shooter is at the targeted speed."""
        if self.shooter_setpoint - \
                ShooterConstants.threshold <= self.shooter_encoder_top.getVelocity() < self.shooter_setpoint + \
                ShooterConstants.threshold:
            return True
        elif self.shooter_setpoint == 0:
            return True
        else:
            return False

    def get_at_angle(self) -> bool:
        """Check up shooter is at the targeted angle."""
        if self.angle_setpoint - \
                ShooterConstants.threshold_ang <= self.encoder.getPosition() <= self.angle_setpoint + \
                ShooterConstants.threshold_ang:
            return True
        elif self.angle_setpoint == (self.angle_setpoints["stow"] + self.trim) and self.angle_setpoint - 0.03 <= \
                self.encoder.getPosition() <= self.angle_setpoint + 0.03:
            return True
        elif self.angle_setpoint == (self.angle_setpoints["subwoofer"] + self.trim) and self.angle_setpoint - 0.02 <= \
                self.encoder.getPosition() <= self.angle_setpoint + 0.02:
            return True
        else:
            return False

    def get_ready_to_shoot(self) -> bool:
        """Check if the shooter has met the prerequisites to fire."""
        if self.get_at_angle() and self.get_at_speed():
            return True
        else:
            return False

    def zero_out(self) -> None:
        """Set the shooter speed to zero."""
        self.shooter_top.set(0)
        self.shooter_bottom.set(0)
        self.feeder.set(0)
        self.shooter_setpoint = 0

    def set_angle(self, angle: float) -> None:
        """Set the angle of the shooter."""
        # if angle != self.angle_setpoints["stow"]:
        self.angle_setpoint = angle + self.trim
        self.angle_pid.setReference(angle + self.trim, CANSparkMax.ControlType.kPosition)
        # else:
        #     self.angle_setpoint = angle
        #     self.angle_pid.setReference(angle, CANSparkMax.ControlType.kPosition)

    def set_known_setpoint(self, setpoint: str) -> None:
        """Set the shooter to a known state."""
        self.spin_up(self.shooter_setpoints[setpoint])
        self.set_angle(self.angle_setpoints[setpoint])

    def set_unknown_setpoint(self, angle: float, speed: float) -> None:
        self.spin_up(speed)
        self.set_angle(angle)

    def get_current_spiked(self) -> bool:
        """Detect if the current has spiked in the shooter."""
        if self.shooter_top.getOutputCurrent() > ShooterConstants.threshold_fired:
            return True
        else:
            return False

    def eject(self) -> None:
        """Manually eject game pieces out the intake."""
        self.feeder.set(-1)
        self.shooter_top.set(-0.2)
        self.shooter_bottom.set(-0.2)

    def get_note_shot(self) -> bool:
        if self.shooter_top.getOutputCurrent() >= 55 or self.shooter_bottom.getOutputCurrent() >= 55:
            return True
        else:
            return False

    def periodic(self) -> None:
        """Any periodic routines for the shooter."""
        SmartDashboard.putNumber("Top Shooter Speed", self.shooter_encoder_top.getVelocity())
        SmartDashboard.putNumber("Bottom Shooter Speed", self.shooter_encoder_bottom.getVelocity())
        SmartDashboard.putNumber("Target Shooter Speed", self.shooter_setpoint)
        SmartDashboard.putNumber("Shooter Angle", self.encoder.getPosition())
        SmartDashboard.putNumber("Shooter Angle Target Real", self.angle_setpoint)
        SmartDashboard.putNumber("Current Trim", self.trim)
        SmartDashboard.putBoolean("Shooter At Setpoint", self.get_ready_to_shoot())
        SmartDashboard.putBoolean("Shooter Up to Speed", self.get_at_speed())
        SmartDashboard.putBoolean("Shooter At Angle", self.get_at_angle())
        SmartDashboard.putNumber("Shooter Current Draw (Top)", self.shooter_top.getOutputCurrent())
        SmartDashboard.putNumber("Shooter Current Draw (Bottom)", self.shooter_bottom.getOutputCurrent())
        SmartDashboard.putBoolean("Note shot?", self.get_note_shot())
        # SmartDashboard.putNumber("Lower Bound", self.shooter_setpoint - ShooterConstants.threshold)
        # SmartDashboard.putNumber("Upper Bound", self.shooter_setpoint + ShooterConstants.threshold)

    def tuning_toggler(self, on: bool) -> None:
        if on:
            self.spin_up(4000)
            self.feeder.set(0.25)
        else:
            self.spin_up(0)
            self.feeder.set(0)

