import wpilib.simulation as sim
from wpilib import RobotController, DriverStation

from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations

from pyfrc.physics.core import PhysicsInterface
from phoenix6 import unmanaged

import typing

if typing.TYPE_CHECKING:
    from robot import Robot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        self.physics_controller = physics_controller

        self.motor_1_sim = sim.DCMotorSim(DCMotor.krakenX60(1), 1, 0.01)
        self.talon_1_sim = robot.m_robotcontainer.motors.motor_1.sim_state

        self.motor_2_sim = sim.DCMotorSim(DCMotor.krakenX60(1), 1, 0.01)
        self.talon_2_sim = robot.m_robotcontainer.motors.motor_2.sim_state

    def update_sim(self, now: float, tm_diff: float):
        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)

        self.talon_1_sim.set_supply_voltage(RobotController.getBatteryVoltage())
        self.talon_2_sim.set_supply_voltage(RobotController.getBatteryVoltage())

        self.motor_1_sim.setInputVoltage(self.talon_1_sim.motor_voltage)
        self.motor_2_sim.setInputVoltage(self.talon_2_sim.motor_voltage)

        self.motor_1_sim.update(tm_diff)
        self.motor_2_sim.update(tm_diff)

        self.talon_1_sim.set_raw_rotor_position(radiansToRotations(self.motor_1_sim.getAngularPosition()))
        self.talon_2_sim.set_raw_rotor_position(radiansToRotations(self.motor_2_sim.getAngularPosition()))

        self.talon_1_sim.set_rotor_velocity(radiansToRotations(self.motor_1_sim.getAngularVelocity()))
        self.talon_2_sim.set_rotor_velocity(radiansToRotations(self.motor_2_sim.getAngularVelocity()))
