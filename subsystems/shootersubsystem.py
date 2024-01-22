import commands2
from rev import CANSparkMax, SparkMaxAbsoluteEncoder
from constants import ShooterConstants
from wpilib import SmartDashboard


class ShooterSubsystem(commands2.Subsystem):

    shooter_setpoints = {"stow": 0, "subwoofer": 0, "podium": 0, "readied": 0}
    angle_setpoints = {"stow": 0, "subwoofer": 0, "podium": 0, "readied": 0}

    def __init__(self) -> None:
        super().__init__()
        self.shooter_top = CANSparkMax(33, CANSparkMax.MotorType.kBrushless)
        self.shooter_bottom = CANSparkMax(34, CANSparkMax.MotorType.kBrushless)
        self.shooter_top.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.shooter_bottom.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.shooter_top.setSmartCurrentLimit(ShooterConstants.current_limit)
        self.shooter_bottom.setSmartCurrentLimit(ShooterConstants.current_limit)
        self.shooter_bottom.follow(self.shooter_top, True)
        self.shooter_pid = self.shooter_top.getPIDController()
        self.shooter_pid.setFF(ShooterConstants.shooter_kFF)
        self.shooter_pid.setP(ShooterConstants.shooter_kP)
        self.shooter_pid.setD(ShooterConstants.shooter_kD)
        self.shooter_encoder = self.shooter_top.getEncoder()

        self.angler = CANSparkMax(38, CANSparkMax.MotorType.kBrushless)
        self.encoder = self.angler.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.angler.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.angle_pid = self.angler.getPIDController()
        self.angle_pid.setFeedbackDevice(self.encoder)
        self.angle_pid.setP(ShooterConstants.angle_kP)
        self.angle_pid.setOutputRange(-0.5, 0.5)  # TODO disable this once it's tuned.

        self.feeder = CANSparkMax(32, CANSparkMax.MotorType.kBrushless)
        self.feeder.setIdleMode(CANSparkMax.IdleMode.kBrake)

        self.shooter_top.burnFlash()
        self.shooter_bottom.burnFlash()
        self.angler.burnFlash()
        self.feeder.burnFlash()

        self.shooter_setpoint = 0
        self.angle_setpoint = 0

    def shoot(self) -> None:
        """Advance Note into shooter wheels."""
        self.feeder.set(ShooterConstants.feeder_speed)

    def spin_up(self, speed: float) -> None:
        """Spin up shooter."""
        self.shooter_pid.setReference(speed, CANSparkMax.ControlType.kVelocity)
        self.shooter_setpoint = speed

    def get_at_speed(self) -> bool:
        """Check up shooter is at the targeted speed."""
        if self.shooter_setpoint - \
                ShooterConstants.threshold <= self.shooter_encoder.getVelocity() <= self.shooter_setpoint + \
                ShooterConstants.threshold:
            return True
        else:
            return False

    def get_at_angle(self) -> bool:
        """Check up shooter is at the targeted angle."""
        if self.angle_setpoint - \
                ShooterConstants.threshold_ang <= self.encoder.getPosition() <= self.angle_setpoint + \
                ShooterConstants.threshold_ang:
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
        self.shooter_setpoint = 0

    def set_angle(self, angle: float) -> None:
        """Set the angle of the shooter."""
        self.angle_pid.setReference(angle, CANSparkMax.ControlType.kPosition)
        self.angle_setpoint = angle

    def set_known_setpoint(self, setpoint: str) -> None:
        """Set the shooter to a known state."""
        self.spin_up(self.shooter_setpoints[setpoint])
        self.set_angle(self.angle_setpoints[setpoint])

    def get_current_spiked(self) -> bool:
        """Detect if the current has spiked in the shooter."""
        if self.shooter_top.getOutputCurrent() > ShooterConstants.threshold_fired:
            return True
        else:
            return False

    def periodic(self) -> None:
        SmartDashboard.putNumber("Roller Speed", self.shooter_encoder.getVelocity())
        SmartDashboard.putNumber("Shooter Angle", self.encoder.getPosition())
