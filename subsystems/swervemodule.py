from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.filter import SlewRateLimiter
from rev import CANSparkMax
from ctre.sensors import CANCoder, AbsoluteSensorRange, CANCoderStatusFrame
from constants import DriveConstants
import math


class SwerveModule:
    driveMotor: CANSparkMax
    rotateMotor: CANSparkMax
    encoder: CANCoder

    def __init__(self, dm: CANSparkMax, rm: CANSparkMax, enc, mod_offset, turn_invert, drive_invert):
        # Connect the passed items to default variables.
        self.driveMotor = dm
        self.rotateMotor = rm
        # The following line is for the swap to REV through bore encoders.
        # self.encoder = rm.getAbsoluteEncoder()
        self.encoder = enc
        self.drive_encoder = self.driveMotor.getEncoder()

        # Set several config items relating to encoders.
        self.drive_encoder.setVelocityConversionFactor(DriveConstants.d_velocity_conversion_factor)
        self.drive_encoder.setPositionConversionFactor(DriveConstants.d_position_conversion_factor)

        self.encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20)
        self.encoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 20)
        # self.encoder.configSensorDirection(not turn_invert)
        self.encoder.setPositionToAbsolute()
        self.encoder.configMagnetOffset(mod_offset)
        self.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)

        # Invert motors based on the passthrough on instantiation.
        self.driveMotor.setInverted(drive_invert)
        self.rotateMotor.setInverted(turn_invert)
        # print("HELLO, here's this encoder's inversion!" + str(self.encoder.getAllConfigs))

        # Set initial state as pointed straight forward @ zero speed.
        self._requested_turn = 0
        self._requested_speed = 0

        # Instantiate PID controllers for azimuth and drive. Set azimuth controller to work in circular frame.
        self._drive_pid_controller = PIDController(DriveConstants.drive_controller_PID[0],
                                                   DriveConstants.drive_controller_PID[1],
                                                   DriveConstants.drive_controller_PID[2])
        self._rotate_pid_controller = PIDController(DriveConstants.azimuth_controller_PID[0],
                                                    DriveConstants.azimuth_controller_PID[1],
                                                    DriveConstants.azimuth_controller_PID[2])
        self._rotate_pid_controller.enableContinuousInput(-math.pi, math.pi)

        # Add feed forward for drive controller. These constants were grabbed from 1678, need tuned.
        self.drive_feed_forward = SimpleMotorFeedforwardMeters(DriveConstants.drive_controller_FF[0],
                                                               DriveConstants.drive_controller_FF[1],
                                                               DriveConstants.drive_controller_FF[2])

        # Set additional constraints on each SPARK MAX controller, including open & closed loop ramp rates, current
        # limits, and idle mode. Then burn to flash memory on each controller.
        rm.setClosedLoopRampRate(DriveConstants.closed_loop_ramp)
        rm.setOpenLoopRampRate(DriveConstants.open_loop_ramp)
        dm.setClosedLoopRampRate(DriveConstants.closed_loop_ramp)
        dm.setOpenLoopRampRate(DriveConstants.open_loop_ramp)
        rm.setSmartCurrentLimit(DriveConstants.azimuth_current_limit)
        dm.setSmartCurrentLimit(DriveConstants.drive_current_limit)
        rm.setIdleMode(CANSparkMax.IdleMode.kBrake)
        dm.setIdleMode(CANSparkMax.IdleMode.kBrake)
        rm.burnFlash()
        dm.burnFlash()

        self.drive_filter = SlewRateLimiter(DriveConstants.slew_rate_drive)
        # self.turn_filter = SlewRateLimiter(DriveConstants.slew_rate_turn)

    def get_state(self):
        """Get the current SwerveModuleState object."""
        return SwerveModuleState(self.drive_encoder.getVelocity(), Rotation2d(math.radians(
            self.encoder.getAbsolutePosition())))

    def get_position(self):
        """Get the current SwerveModulePosition object."""
        return SwerveModulePosition(self.drive_encoder.getPosition(), Rotation2d(math.radians(
            self.encoder.getAbsolutePosition())))

    def set_desired_state(self, desired_state):
        """Set the targets for the Swerve Module to meet."""
        # Optimize module orientation based on current orientation. Subroutine courtesy of Team 461.
        state = self.optimize_module(desired_state)

        # Calculate the PID and FF controllers for the module's motors.
        drive_output = self._drive_pid_controller.calculate(self.drive_encoder.getVelocity(), state.speed)
        drive_ff = self.drive_feed_forward.calculate(state.speed)
        rotate_output = self._rotate_pid_controller.calculate(math.radians(self.encoder.getAbsolutePosition()),
                                                              state.angle.radians())

        # Set the voltage of each motor based on the PID and FF values. Will likely deprecate when swapping to
        # REV through bore encoder.
        # self.driveMotor.setVoltage(drive_output + drive_ff)
        self.rotateMotor.setVoltage(rotate_output)

        self.driveMotor.setVoltage(self.drive_filter.calculate(drive_output + drive_ff))
        # self.rotateMotor.setVoltage(self.turn_filter.calculate(rotate_output))

    def reset_encoders(self):
        """Reset the drive encoder to its zero position. Should not be used in normal software as it results in
        irretrievable odometry data loss."""
        self.drive_encoder.setPosition(0)

    def optimize_module(self, desired_state: SwerveModuleState) -> SwerveModuleState:
        """Python port of Team 461's module optimization code. I could comment all this, but they didn't, and I had
        to figure out what it did on my own. So you have to do that too."""
        inverted = False
        desired_degrees = desired_state.angle.degrees()  # 360.0
        if desired_degrees < 0.0:
            desired_degrees += 360.0

        current_degrees = self.encoder.getAbsolutePosition()
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
