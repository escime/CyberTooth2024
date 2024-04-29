import commands2
from commands2 import button, cmd, SequentialCommandGroup
import wpilib.simulation

from constants import OIConstants
from subsystems.ledsubsystem import LEDs
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.motorsubsystem import MotorSubsystem
from subsystems.encodersubsystem import EncoderSubsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager, Timer
from commands.default_leds import DefaultLEDs
from helpers.custom_hid import CustomHID
from pathplannerlib.auto import NamedCommands, PathPlannerAuto


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.timer = Timer()
        self.timer.start()
        if wpilib.RobotBase.isReal():
            print("Not a simulation, logging enabled!")
            DataLogManager.start()
            DriverStation.startDataLog(DataLogManager.getLog(), True)
        else:
            print("Simulated, logging disabled.")

        # Instantiate subsystems using their constructors if tuning mode is disabled.
        self.leds = LEDs(0, 20, 1, 0.03, "GRB", self.timer)
        self.vision_system = VisionSubsystem(self.timer)
        self.motors = MotorSubsystem()
        self.encoders = EncoderSubsystem()

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(OIConstants.kDriverControllerPort, "xbox")
        DriverStation.silenceJoystickConnectionWarning(True)

        # Set default subsystem commands.
        self.leds.setDefaultCommand(DefaultLEDs(self.leds))

        # Register commands for PathPlanner.
        self.registerCommands()

        # Setup for all event-trigger commands.
        self.configureTriggersDefault()

        # Setup autonomous selector on the dashboard.
        self.m_chooser = SendableChooser()
        self.auto_names = ["Test"]
        self.m_chooser.setDefaultOption("DoNothing", "DoNothing")
        for x in self.auto_names:
            self.m_chooser.addOption(x, x)

        SmartDashboard.putData("Auto Select", self.m_chooser)

        self.motors.setDefaultCommand(
            SequentialCommandGroup(
                cmd.run(lambda: self.motors.set_motor_duty_cycle(1, self.driver_controller_raw.get_axis("LY", 0.05)),
                        self.motors),
                cmd.run(lambda: self.motors.set_motor_duty_cycle(2, self.driver_controller_raw.get_axis("LX", 0.05)),
                        self.motors)
            )
        )

    def configureTriggersDefault(self) -> None:
        """Used to set up any commands that trigger when a measured event occurs."""
        print("No triggers configured!")

    def getAutonomousCommand(self) -> commands2.cmd:
        """Use this to pass the autonomous command to the main Robot class.
        Returns the command to run in autonomous
        """
        if self.m_chooser.getSelected() == "DoNothing":
            return None
        else:
            selected_auto = None
            for y in self.auto_names:
                if self.m_chooser.getSelected() == y:
                    try:
                        selected_auto = PathPlannerAuto(y)
                    except FileNotFoundError:
                        selected_auto = None
            return selected_auto

    def registerCommands(self):
        NamedCommands.registerCommand("rainbow_leds", commands2.cmd.run(lambda: self.leds.rainbow_shift(), self.leds))
        NamedCommands.registerCommand("flash_green",
                                      commands2.cmd.run(lambda: self.leds.flash_color([255, 0, 0], 2), self.leds))
        NamedCommands.registerCommand("flash_red",
                                      commands2.cmd.run(lambda: self.leds.flash_color([0, 255, 0], 2), self.leds))
        NamedCommands.registerCommand("flash_blue",
                                      commands2.cmd.run(lambda: self.leds.flash_color([0, 0, 255], 2), self.leds))
        NamedCommands.registerCommand("flash_yellow",
                                      commands2.cmd.run(lambda: self.leds.flash_color([225, 255, 0], 2), self.leds))
        NamedCommands.registerCommand("flash_purple",
                                      commands2.cmd.run(lambda: self.leds.flash_color([50, 149, 168], 2), self.leds))
