import commands2
from constants import IntakeConstants
from rev import CANSparkMax


class IntakeSubsystem(commands2.Subsystem):

    def __init__(self) -> None:
        super().__init__()
        self.motor = CANSparkMax(IntakeConstants.motor_id, CANSparkMax.MotorType.kBrushless)
        self.follower = CANSparkMax(IntakeConstants.follower_id, CANSparkMax.MotorType.kBrushless)
        self.motor.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.follower.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.motor.setSmartCurrentLimit(IntakeConstants.current_limit)
        self.follower.setSmartCurrentLimit(IntakeConstants.current_limit)
        self.follower.follow(self.motor, True)
        self.motor.burnFlash()
        self.follower.burnFlash()

    def intake(self, voltage: float):
        """Power the motor without changing deploy state."""
        self.motor.setVoltage(voltage)
