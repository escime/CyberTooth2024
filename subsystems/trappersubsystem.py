import commands2
from constants import TrapperConstants
from rev import CANSparkMax
from wpilib import DigitalInput, SmartDashboard


class TrapperSubsystem(commands2.Subsystem):

    setpoints = {"stage": 0, "trap": 0, "amp": 0, "stow": 0}

    def __init__(self) -> None:
        super().__init__()
        self.trap = CANSparkMax(35, CANSparkMax.MotorType.kBrushless)
        self.arm = CANSparkMax(36, CANSparkMax.MotorType.kBrushless)
        self.climb = CANSparkMax(37, CANSparkMax.MotorType.kBrushless)

        self.trap.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.arm.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.climb.setIdleMode(CANSparkMax.IdleMode.kBrake)

        self.arm.setSmartCurrentLimit(TrapperConstants.arm_limit)
        self.climb.setSmartCurrentLimit(TrapperConstants.climb_limit)

        self.arm_pid = self.arm.getPIDController()
        self.arm_pid.setP(TrapperConstants.kP)
        self.arm_pid.setOutputRange(-0.5, 0.5)  # TODO tune this later

        self.trap.burnFlash()
        self.arm.burnFlash()
        self.climb.burnFlash()

        self.sensor = DigitalInput(0)
        self.arm_encoder = self.arm.getEncoder()
        self.arm_encoder.setPosition(0)
        self.climb_encoder = self.climb.getEncoder()
        self.climb_encoder.setPosition(0)

        self.arm_setpoint = "stow"

    def get_note_acquired(self) -> bool:
        if self.sensor.get():
            return False
        else:
            return True

    def advance_to_trapper(self) -> None:
        if not self.get_note_acquired():
            self.trap.set(TrapperConstants.trap_speed)
        else:
            self.trap.set(0)

    def advance(self) -> None:
        self.trap.set(TrapperConstants.trap_speed)

    def score_in_amp(self) -> None:
        self.trap.set(TrapperConstants.trap_speed * -1)

    def manual_trap(self, speed: float) -> None:
        self.trap.set(speed)

    def stow(self) -> None:
        self.trap.set(0)

    def set_arm(self, setpoint: str) -> None:
        self.arm_pid.setReference(self.setpoints[setpoint], CANSparkMax.ControlType.kPosition)
        self.arm_setpoint = setpoint

    def run_climb(self, speed: float) -> None:
        self.climb.set(speed)

    def set_climb_stage_1(self) -> None:
        if self.arm_setpoint != "stage":
            self.set_arm("stage")
        if self.climb_encoder.getPosition() < TrapperConstants.climber_preset:
            self.run_climb(1)
        else:
            self.run_climb(0)

    def set_climb_stage_2(self) -> None:
        if self.arm_setpoint != "trap":
            self.set_arm("trap")
        if self.climb_encoder.getPosition() < TrapperConstants.climber_preset_2:
            self.run_climb(1)
        else:
            self.run_climb(0)

    def periodic(self) -> None:
        SmartDashboard.putNumber("Arm Position", self.arm_encoder.getPosition())
        SmartDashboard.putNumber("Climber Position", self.climb_encoder.getPosition())
