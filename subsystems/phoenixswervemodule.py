from commands2 import Subsystem
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import VelocityVoltage, PositionVoltage
from phoenix6.configs import MotorOutputConfigs, TalonFXConfiguration
from phoenix6.signals import InvertedValue, FeedbackSensorSourceValue
from constants import TunerConstants
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.geometry import Rotation2d


class PhoenixSwerveModule(Subsystem):
    # TODO deal with gear ratios for steering and driving, RIP FusedCANCoder.

    def __init__(self, drive_id: int, steer_id: int, encoder_id: int, invert_drive: bool, invert_steer: bool):
        super().__init__()
        # Create TalonFX objects.
        self.drive_motor = TalonFX(drive_id, TunerConstants.k_can_bus_name)
        self.steer_motor = TalonFX(steer_id, TunerConstants.k_can_bus_name)

        # Create CANcoder object.
        self.steer_encoder = CANcoder(encoder_id, TunerConstants.k_can_bus_name)

        # Create drive and steer motor configuration objects.
        self.drive_motor_config = self.drive_motor.configurator
        self.steer_motor_config = self.steer_motor.configurator

        # Modify configs to invert based on module instantiation.
        drive_motor_motor_out_config = MotorOutputConfigs()
        if invert_drive:
            drive_motor_motor_out_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        else:
            drive_motor_motor_out_config.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        steer_motor_motor_out_config = MotorOutputConfigs()
        if invert_steer:
            steer_motor_motor_out_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        else:
            steer_motor_motor_out_config.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        # Set steer motor to utilize the CANcoder as a remote sensor.
        # TODO Switch to fusedCANCoder if on PRO license.
        steer_fx_cfg = TalonFXConfiguration()
        steer_fx_cfg.feedback.feedback_remote_sensor_id = encoder_id
        steer_fx_cfg.feedback_sensor_source = FeedbackSensorSourceValue.REMOTE_CANCODER

        # Apply configs for inversion, remote sensor, and drive tuning.
        self.drive_motor_config.apply(drive_motor_motor_out_config)
        self.drive_motor_config.apply(TunerConstants.drive_gains)
        self.steer_motor_config.apply(steer_motor_motor_out_config)
        self.steer_motor_config.apply(steer_fx_cfg)
        self.steer_motor_config.apply(TunerConstants.steer_gains)

        # Create objects for the position and velocity feeds for each motor.
        self.dm_pos = self.drive_motor.get_position()
        self.sm_pos = self.steer_motor.get_position()
        self.dm_vel = self.drive_motor.get_velocity()
        self.sm_vel = self.steer_motor.get_velocity()

        # Set default state of each motor to 0 position. For the steer motor, this will snap the wheel forward on
        # startup. For the drive motor, this will tell it not to move on startup.
        self.drive_control = VelocityVoltage(0, enable_foc=TunerConstants.enable_foc).with_slot(0)
        self.steer_control = PositionVoltage(0, enable_foc=TunerConstants.enable_foc).with_slot(0)

    def get_state(self) -> SwerveModuleState:
        """Get the WPILIB SwerveModuleState from the module."""
        self.dm_vel.wait_for_update(0.01)
        self.sm_pos.wait_for_update(0.01)
        return SwerveModuleState(self.dm_vel.value_as_double, Rotation2d(self.sm_pos.value_as_double))

    def get_position(self) -> SwerveModulePosition:
        """Get the WPILIB SwerveModulePosition from the module."""
        self.dm_pos.wait_for_update(0.01)
        self.sm_pos.wait_for_update(0.01)
        return SwerveModulePosition(self.dm_pos.value_as_double, Rotation2d(self.sm_pos.value_as_double))

    def set_desired_state(self, desired_state: SwerveModuleState) -> None:
        """Set the desired SwerveModuleState by optimizing and then setting the Talons' control state."""
        state = self.optimize_module(desired_state)

        self.drive_motor.set_control(self.drive_control.with_velocity(state.speed / TunerConstants.k_drive_gear_ratio))

        self.steer_motor.set_control(self.steer_control.with_position(state.angle.degrees()))

    def reset_drive_encoder(self) -> None:
        """Reset the drive encoder to zero."""
        self.drive_motor.set_position(0)

    def optimize_module(self, desired_state: SwerveModuleState) -> SwerveModuleState:
        """Optimize the module state. Credit to @frc461."""
        inverted = False
        desired_degrees = desired_state.angle.degrees()  # 360.0
        if desired_degrees < 0.0:
            desired_degrees += 360.0

        current_degrees = self.sm_pos.value_as_double
        current_mod = current_degrees % 360.0
        if current_mod < 0.0:
            current_mod += 360

        if 90.0 < abs(current_mod - desired_degrees) <= 270.0:
            inverted = True
            desired_degrees -= 180.0

        delta_angle = desired_degrees - current_mod
        if delta_angle < 0.0:
            delta_angle += 360.0

        ccw_angle = delta_angle
        cw_angle = delta_angle - 360.0

        if abs(ccw_angle) < abs(cw_angle):
            desired_degrees = ccw_angle
        else:
            desired_degrees = cw_angle

        magnitude = desired_state.speed

        if inverted:
            magnitude = magnitude * -1

        desired_degrees += current_degrees

        return SwerveModuleState(magnitude, Rotation2d.fromDegrees(desired_degrees))
