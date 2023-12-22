import commands2
from rev import CANSparkMax
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from constants import ModuleConstants, DriveConstants


class TuningSubsystem(commands2.SubsystemBase):

    ks_found = False
    voltage = 0
    ks = 0
    ks_found_list = [False, False, False, False]
    voltages = [0, 0, 0, 0]
    ks_list = [0, 0, 0, 0]
    stabilized = False
    last_vel = 0
    kv = 0
    stabilized_list = [False, False, False, False]
    last_vel_list = [0, 0, 0, 0]
    kv_list = [0, 0, 0, 0]
    kv_avg = 0

    def __init__(self, drive: bool, single_motor: bool, canid: int) -> None:
        super().__init__()
        if single_motor:
            self.single_motor = CANSparkMax(canid, CANSparkMax.MotorType.kBrushless)
            self.single_motor_enc = self.single_motor.getEncoder()

        if drive:
            self.fl_drive = CANSparkMax(ModuleConstants.fl_drive_id, CANSparkMax.MotorType.kBrushless)
            self.fr_drive = CANSparkMax(ModuleConstants.fr_drive_id, CANSparkMax.MotorType.kBrushless)
            self.bl_drive = CANSparkMax(ModuleConstants.bl_drive_id, CANSparkMax.MotorType.kBrushless)
            self.br_drive = CANSparkMax(ModuleConstants.br_drive_id, CANSparkMax.MotorType.kBrushless)

            self.fl_turn = CANSparkMax(ModuleConstants.fl_turn_id, CANSparkMax.MotorType.kBrushless)
            self.fr_turn = CANSparkMax(ModuleConstants.fr_turn_id, CANSparkMax.MotorType.kBrushless)
            self.bl_turn = CANSparkMax(ModuleConstants.bl_turn_id, CANSparkMax.MotorType.kBrushless)
            self.br_turn = CANSparkMax(ModuleConstants.br_turn_id, CANSparkMax.MotorType.kBrushless)

            self.fl_drive_enc = self.fl_drive.getEncoder()
            self.fr_drive_enc = self.fr_drive.getEncoder()
            self.bl_drive_enc = self.bl_drive.getEncoder()
            self.br_drive_enc = self.br_drive.getEncoder()

            self.fl_turn_enc = self.fl_turn.getEncoder()
            self.fr_turn_enc = self.fr_turn.getEncoder()
            self.bl_turn_enc = self.bl_turn.getEncoder()
            self.br_turn_enc = self.br_turn.getEncoder()

            self.fl_drive_enc.setVelocityConversionFactor(DriveConstants.d_velocity_conversion_factor)
            self.fr_drive_enc.setVelocityConversionFactor(DriveConstants.d_velocity_conversion_factor)
            self.bl_drive_enc.setVelocityConversionFactor(DriveConstants.d_velocity_conversion_factor)
            self.br_drive_enc.setVelocityConversionFactor(DriveConstants.d_velocity_conversion_factor)
            self.fl_drive_enc.setPositionConversionFactor(DriveConstants.d_position_conversion_factor)
            self.fr_drive_enc.setPositionConversionFactor(DriveConstants.d_position_conversion_factor)
            self.bl_drive_enc.setPositionConversionFactor(DriveConstants.d_position_conversion_factor)
            self.br_drive_enc.setPositionConversionFactor(DriveConstants.d_position_conversion_factor)

            self.fl_drive.setInverted(True)
            self.fr_drive.setInverted(True)
            self.bl_drive.setInverted(True)
            self.br_drive.setInverted(True)
            self.fl_turn.setInverted(True)
            self.fr_turn.setInverted(True)
            self.bl_turn.setInverted(True)
            self.br_turn.setInverted(True)

            self.fl_turn_pid = PIDController(1, 0, 0)
            self.fr_turn_pid = PIDController(1, 0, 0)
            self.bl_turn_pid = PIDController(1, 0, 0)
            self.br_turn_pid = PIDController(1, 0, 0)

            self.fl_drive.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.fr_drive.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.bl_drive.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.br_drive.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.fl_turn.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.fr_turn.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.bl_turn.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.br_turn.setIdleMode(CANSparkMax.IdleMode.kBrake)

            self.fl_drive.burnFlash()
            self.fr_drive.burnFlash()
            self.bl_drive.burnFlash()
            self.br_drive.burnFlash()
            self.fl_turn.burnFlash()
            self.fr_turn.burnFlash()
            self.bl_turn.burnFlash()
            self.br_turn.burnFlash()

    def id_ks_sm(self, threshold: float) -> None:
        """Identify the kS term for a single motor."""
        if self.single_motor_enc.getPosition() > threshold:
            self.ks_found = True
            self.ks = self.voltage
            self.single_motor.setVoltage(0)
        else:
            self.voltage += 0.01
            self.single_motor.setVoltage(self.voltage)
        SmartDashboard.putNumber("Calculated kS", self.ks)
        print(self.ks)

    def id_ks_dt(self, threshold: float) -> None:
        """Identify the kS term for the drivetrain."""
        self.fl_turn.setVoltage(self.fl_turn_pid.calculate(self.fl_turn_enc.getPosition(), 0))
        self.fr_turn.setVoltage(self.fr_turn_pid.calculate(self.fr_turn_enc.getPosition(), 0))
        self.bl_turn.setVoltage(self.bl_turn_pid.calculate(self.bl_turn_enc.getPosition(), 0))
        self.br_turn.setVoltage(self.br_turn_pid.calculate(self.br_turn_enc.getPosition(), 0))

        if not self.ks_found_list[0]:
            if self.fl_drive_enc.getPosition() > threshold:
                self.ks_found_list[0] = True
                self.ks_list[0] = self.voltages[0]
                self.fl_drive.setVoltage(0)
            else:
                self.fl_drive.setVoltage(self.voltages[0])
                self.voltages[0] += 0.01
        if not self.ks_found_list[1]:
            if self.fr_drive_enc.getPosition() > threshold:
                self.ks_found_list[1] = True
                self.ks_list[1] = self.voltages[1]
                self.fr_drive.setVoltage(0)
            else:
                self.fr_drive.setVoltage(self.voltages[1])
                self.voltages[1] += 0.01
        if not self.ks_found_list[2]:
            if self.bl_drive_enc.getPosition() > threshold:
                self.ks_found_list[2] = True
                self.ks_list[2] = self.voltages[2]
                self.bl_drive.setVoltage(0)
            else:
                self.bl_drive.setVoltage(self.voltages[2])
                self.voltages[2] += 0.01
        if not self.ks_found_list[3]:
            if self.br_drive_enc.getPosition() > threshold:
                self.ks_found_list[3] = True
                self.ks_list[3] = self.voltages[3]
                self.br_drive.setVoltage(0)
            else:
                self.br_drive.setVoltage(self.voltages[3])
                self.voltages[3] += 0.01
        SmartDashboard.putNumber("Calculated kS", max(self.ks_list))
        print(max(self.ks_list))

    def id_kv_sm(self, voltage):
        """Identify the kV term for a single motor."""
        if not self.stabilized:
            self.single_motor.setVoltage(voltage)
            if self.single_motor_enc.getVelocity() < self.last_vel:
                self.stabilized = True
            self.last_vel = self.single_motor_enc.getVelocity()
        else:
            self.kv = self.last_vel / voltage
            self.single_motor.setVoltage(0)
        SmartDashboard.putNumber("Calculated kV", self.kv)
        print(self.kv)

    def id_kv_dt(self, voltage: float) -> None:
        """Identify the kV term for the drivetrain."""
        if False in self.stabilized_list:
            self.fl_drive.setVoltage(voltage)
            self.fr_drive.setVoltage(voltage)
            self.bl_drive.setVoltage(voltage)
            self.br_drive.setVoltage(voltage)
            if self.fl_drive_enc.getVelocity() < self.last_vel_list[0]:
                self.stabilized_list[0] = True
            if self.fr_drive_enc.getVelocity() < self.last_vel_list[1]:
                self.stabilized_list[1] = True
            if self.bl_drive_enc.getVelocity() < self.last_vel_list[2]:
                self.stabilized_list[2] = True
            if self.br_drive_enc.getVelocity() < self.last_vel_list[3]:
                self.stabilized_list[3] = True
            self.last_vel_list[0] = self.fl_drive_enc.getVelocity()
            self.last_vel_list[1] = self.fr_drive_enc.getVelocity()
            self.last_vel_list[2] = self.bl_drive_enc.getVelocity()
            self.last_vel_list[3] = self.br_drive_enc.getVelocity()
        else:
            self.fl_drive.setVoltage(0)
            self.fr_drive.setVoltage(0)
            self.bl_drive.setVoltage(0)
            self.br_drive.setVoltage(0)

            self.kv_list[0] = self.last_vel_list[0] / voltage
            self.kv_list[1] = self.last_vel_list[1] / voltage
            self.kv_list[2] = self.last_vel_list[2] / voltage
            self.kv_list[3] = self.last_vel_list[3] / voltage
            self.kv_avg = sum(self.kv_list) / len(self.kv_list)

        SmartDashboard.putNumber("Calculated kV", self.kv_avg)
        print(self.kv_avg)

    def set_all_zero(self) -> None:
        self.fl_drive.setVoltage(0)
        self.fr_drive.setVoltage(0)
        self.bl_drive.setVoltage(0)
        self.br_drive.setVoltage(0)
        self.fl_turn.setVoltage(0)
        self.fr_turn.setVoltage(0)
        self.bl_turn.setVoltage(0)
        self.br_turn.setVoltage(0)
        self.single_motor.setVoltage(0)
