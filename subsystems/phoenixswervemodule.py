import math

from phoenix6.hardware import TalonFX, Pigeon2, CANcoder
from phoenix6.configs import CANcoderConfigurator, CANcoderConfiguration
from phoenix6.configs.config_groups import MagnetSensorConfigs
from phoenix6.controls import VelocityVoltage, PositionVoltage
from constants import TunerConstants
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.geometry import Rotation2d


class PhoenixSwerveModule:

    def __init__(self, drive_id: int, steer_id: int, encoder_id: int, encoder_offset: float, invert: bool):
        self.drive_motor = TalonFX(drive_id, TunerConstants.k_can_bus_name)
        self.steer_motor = TalonFX(steer_id, TunerConstants.k_can_bus_name)
        self.steer_encoder = CANcoder(encoder_id, TunerConstants.k_can_bus_name)

        self.drive_motor.configurator.apply(TunerConstants.drive_gains)
        self.steer_motor.configurator.apply(TunerConstants.steer_gains)

        self.drive_control = VelocityVoltage(0, enable_foc=False).with_slot(0)
        self.steer_control = PositionVoltage(0, enable_foc=False).with_slot(0)

        self.steer_motor.set_position(self.steer_encoder.get_absolute_position().value_as_double *
                                      TunerConstants.k_steer_gear_ratio)

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(self.drive_motor.get_velocity().value,
                                 Rotation2d(math.radians(self.get_steer_angle())))

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drive_motor.get_position().value_as_double / TunerConstants.k_drive_gear_ratio,
                                    Rotation2d(math.radians(self.get_steer_angle())))

    def set_desired_state(self, desired_state: SwerveModuleState) -> None:
        state = self.optimize_module(desired_state)

        self.drive_motor.set_control(self.drive_control.with_velocity(state.speed / TunerConstants.k_drive_gear_ratio))

        self.steer_motor.set_control(self.steer_control.with_position(state.angle.degrees() /
                                                                       TunerConstants.k_steer_gear_ratio))

    def reset_drive_encoder(self) -> None:
        self.drive_motor.set_position(0)

    def get_steer_angle(self) -> float:
        return (self.steer_motor.get_position().value_as_double / TunerConstants.k_steer_gear_ratio) % 360

    def optimize_module(self, desired_state: SwerveModuleState) -> SwerveModuleState:
        """Python port of Team 461's module optimization code. I could comment all this, but they didn't, and I had
        to figure out what it did on my own. So you have to do that too."""
        inverted = False
        desired_degrees = desired_state.angle.degrees()  # 360.0
        if desired_degrees < 0.0:
            desired_degrees += 360.0

        current_degrees = self.get_steer_angle()
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
