import commands2
from constants import IntakeConstants
from rev import CANSparkMax
from wpilib import SmartDashboard


class IntakeSubsystem(commands2.Subsystem):

    def __init__(self) -> None:
        super().__init__()
        self.motor = CANSparkMax(IntakeConstants.motor_id, CANSparkMax.MotorType.kBrushless)
        self.follower = CANSparkMax(IntakeConstants.follower_id, CANSparkMax.MotorType.kBrushless)
        self.motor.setInverted(True)
        self.motor.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.follower.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.motor.setSmartCurrentLimit(IntakeConstants.current_limit)
        self.follower.setSmartCurrentLimit(IntakeConstants.current_limit)
        self.follower.follow(self.motor, False)
        self.motor.burnFlash()
        self.follower.burnFlash()

    def intake(self, voltage: float):
        """Power the intake motors."""
        self.motor.setVoltage(voltage * 12)

    def periodic(self) -> None:
        SmartDashboard.putNumber("Intake Current Draw", self.motor.getOutputCurrent())
