import commands2
from constants import TrapperConstants
from rev import CANSparkMax
from wpilib import DigitalInput, SmartDashboard


class TrapperSubsystem(commands2.Subsystem):

    setpoints = {"stage": 8.3, "trap": 20.5, "amp": 9, "stow": 0, "manual_off": 0, "manual": 0}

    def __init__(self) -> None:
        super().__init__()
        # Configure Motor IDs.
        self.trap = CANSparkMax(35, CANSparkMax.MotorType.kBrushless)
        self.arm = CANSparkMax(36, CANSparkMax.MotorType.kBrushless)
        self.arm.setInverted(True)
        self.climb = CANSparkMax(37, CANSparkMax.MotorType.kBrushless)
        self.climb.setInverted(True)

        # Set motor idle behavior.
        self.trap.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.arm.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.climb.setIdleMode(CANSparkMax.IdleMode.kBrake)

        # Set motor smart current limits.
        self.arm.setSmartCurrentLimit(TrapperConstants.arm_limit)
        self.climb.setSmartCurrentLimit(TrapperConstants.climb_limit)

        # Configure arm PID.
        self.arm_pid = self.arm.getPIDController()
        self.arm_pid.setP(TrapperConstants.kP)
        self.arm_pid.setOutputRange(-1, 1)

        # Setup NOTE detection sensors.
        self.sensor_bottom = DigitalInput(0)
        self.sensor_top = DigitalInput(1)

        # Setup encoders.
        self.arm_encoder = self.arm.getEncoder()
        self.arm_encoder.setPosition(0)
        self.arm_encoder.setPositionConversionFactor(TrapperConstants.positionConversion)
        self.climb_encoder = self.climb.getEncoder()
        self.climb_encoder.setPosition(0)

        # Set arm setpoint to stow at startup.
        self.arm_setpoint = "stow"

        # Tell robot it's not climbing
        self.is_climbing = False

        self.note_acquisition_buffer = [False] * 7

        # self.mech = Mechanism2d(6, 6)
        # self.mech_root = self.mech.getRoot("core", 3, 3)
        # self.mech_arm = self.mech_root.appendLigament("Arm", 3, -180)

        # TODO test if this actually decreases CAN utilization
        self.trap.setControlFramePeriodMs(60)
        self.arm.setControlFramePeriodMs(60)
        self.climb.setControlFramePeriodMs(60)

        # Burn all settings to flash memory on the SPARK Maxes.
        self.trap.burnFlash()
        self.arm.burnFlash()
        self.climb.burnFlash()

    def get_note_acquired(self) -> bool:
        """Check if the robot has a NOTE in the Trapper."""
        if all(self.note_acquisition_buffer):
            return True
        else:
            return False

    def advance_to_trapper(self) -> None:
        """Advance the trapper intake until a NOTE is detected."""
        if not self.get_note_acquired():
            self.trap.set(TrapperConstants.trap_speed)
        else:
            self.trap.set(0)

    def advance(self) -> None:
        """Advance the NOTE to the shooter."""
        self.trap.set(TrapperConstants.trap_speed)

    def score_in_amp(self, inverted: bool) -> None:
        """Score the NOTE in the AMP by running the trap intake backwards."""
        if inverted:
            self.trap.set(TrapperConstants.amp_speed)
        else:
            self.trap.set(TrapperConstants.amp_speed * -1)

    def manual_trap(self, speed: float) -> None:
        """Manually control the speed of the trap intake."""
        self.trap.set(speed)

    def stow(self) -> None:
        """Set the trap intake and arm to stow states."""
        self.trap.set(0)
        self.set_arm("stow")

    def set_arm(self, setpoint: str) -> None:
        """Set the arm to a known setpoint."""
        self.arm_pid.setReference(self.setpoints[setpoint], CANSparkMax.ControlType.kPosition)
        self.arm_setpoint = setpoint

    def run_climb(self, speed: float) -> None:
        """Run the climber at a given speed."""
        self.climb.set(speed)

    def manual_arm(self, speed: float) -> None:
        if self.arm_encoder.getPosition() > (self.setpoints["trap"] + 1) and speed > 0:
            self.arm.set(0)
        elif self.arm_encoder.getPosition() < 0 and speed < 0:
            self.arm.set(0)
        else:
            self.arm.set(speed)
        self.arm_setpoint = "manual"

    def manual_arm_off(self) -> None:
        self.arm.set(0)
        self.arm_setpoint = "manual_off"
        self.arm_pid.setReference(self.arm_encoder.getPosition(), CANSparkMax.ControlType.kPosition)

    def check_arm_position(self):
        if abs(self.arm_encoder.getPosition() - self.setpoints[self.arm_setpoint]) < 0.1:
            return True
        else:
            return False

    def set_climb_stage_1(self) -> None:
        """Preset the trap mechanisms for being under the stage."""
        self.is_climbing = True
        if self.arm_setpoint != "stage":
            self.set_arm("stage")
        if self.climb_encoder.getPosition() < TrapperConstants.climber_preset:
            self.run_climb(1)
        else:
            self.run_climb(0)

    def set_climb_stage_2(self) -> None:
        """Presets the trap mechanisms for being between the chain and the stage."""
        self.is_climbing = True
        if self.arm_setpoint != "trap":
            self.set_arm("trap")
        if self.climb_encoder.getPosition() < TrapperConstants.climber_preset_2:
            self.run_climb(1)
        else:
            self.run_climb(0)

    def periodic(self) -> None:
        """Any periodic routines for the trapper."""
        # SmartDashboard.putNumber("Arm Position", self.arm_encoder.getPosition())
        SmartDashboard.putNumber("Climber Position", self.climb_encoder.getPosition())
        if not self.sensor_bottom.get() and not self.sensor_top.get():
            self.note_acquisition_buffer[0] = True
        else:
            self.note_acquisition_buffer[0] = False
        self.note_acquisition_buffer = self.note_acquisition_buffer[1:] + self.note_acquisition_buffer[:1]
        SmartDashboard.putBoolean("Note Acquired?", self.get_note_acquired())
        # SmartDashboard.putBooleanArray("Note Acquisition Buffer", self.note_acquisition_buffer)
        SmartDashboard.putString("Arm Setpoint", self.arm_setpoint)
        # SmartDashboard.putNumber("Trapper Current Draw", self.trap.getOutputCurrent())

    def reset_climber_zero(self) -> None:
        self.climb_encoder.setPosition(0)

    def stop_climbing(self) -> None:
        self.is_climbing = False
