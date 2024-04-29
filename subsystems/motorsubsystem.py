from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut


class MotorSubsystem(Subsystem):
    """A subsystem for controlling motors on the CUBE."""
    def __init__(self):
        super().__init__()

        self.motor_1 = TalonFX(30, "rio")
        self.motor_2 = TalonFX(31, "rio")

        self.c_motor_1 = DutyCycleOut(0, enable_foc=False)
        self.c_motor_2 = DutyCycleOut(0, enable_foc=False)

    def set_motor_duty_cycle(self, motor: int, duty_cycle: float) -> None:
        if motor == 1:
            self.motor_1.set_control(self.c_motor_1.with_output(duty_cycle))
        elif motor == 2:
            self.motor_2.set_control(self.c_motor_2.with_output(duty_cycle))
        else:
            print("Unable to set motor duty cycle. No programmed motor has that ID.")

    def get_motor_position(self, motor: int) -> float:
        if motor == 1:
            return self.motor_1.get_position().value_as_double
        elif motor == 2:
            return self.motor_2.get_position().value_as_double
        else:
            print("Unable to acquire motor position. No programmed motor has that ID.")
            return 0

    def get_motor_velocity(self, motor: int) -> float:
        if motor == 1:
            vel = self.motor_1.get_velocity()
            vel.wait_for_update(0.1)
            return vel.value_as_double
        elif motor == 2:
            vel = self.motor_2.get_velocity()
            vel.wait_for_update(0.1)
            return vel.value_as_double
        else:
            print("Unable to acquire motor velocity. No programmed motor has that ID.")
            return 0

