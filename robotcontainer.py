import commands2
import wpilib.simulation
from commands2 import button

from constants import OIConstants
from subsystems.ledsubsystem2 import LEDs
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager, LiveWindow, Timer
from commands.shoot_leds import ShootLEDs
from commands.amp_leds import AmpLEDs
from commands.alert_gp_leds import AlertGPLEDs
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
        tuning_setter = False
        self.timer = Timer()
        self.timer.start()
        if wpilib.RobotBase.isReal():
            print("Not a simulation, logging enabled!")
            DataLogManager.start()
            DriverStation.startDataLog(DataLogManager.getLog(), True)
        else:
            print("Simulated, logging disabled.")

        LiveWindow.disableAllTelemetry()

        self.leds = LEDs(self.timer)

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(OIConstants.kDriverControllerPort, "xbox")
        self.operator_controller_raw = CustomHID(OIConstants.kOperatorControllerPort, "xbox")
        DriverStation.silenceJoystickConnectionWarning(True)

        # Register commands for PathPlanner.
        self.registerCommands()

        # Setup for all event-trigger commands.
        self.configureTriggersDefault()

        # Setup autonomous selector on the dashboard.
        self.m_chooser = SendableChooser()
        self.auto_names = ["Test", "MobilityOnly", "ScoreOnly", "A_ScoreMobility", "B_ScoreMobility",
                           "C_ScoreMobility", "A_Score2_Close", "B_Score2_Close", "C_Score2_Close",
                           "A_Score4", "B_Score4", "C_Score4", "A_Score2", "C_Score2", "C_Score3", "A_Score3",
                           "B_Score4_Fast", "B_Score4_Fastest", "B_Score3.5", "A_Score3_Midline",
                           "C_Score3_Midline", "Chaos", "B_Score4.5", "ChoreoTesting", "0SystemsCheck"]
        self.m_chooser.setDefaultOption("DoNothing", "DoNothing")
        for x in self.auto_names:
            self.m_chooser.addOption(x, x)

        SmartDashboard.putData("Auto Select", self.m_chooser)

    def configureTriggersDefault(self) -> None:
        """Used to set up any commands that trigger when a measured event occurs."""
        # Start an AMPLIFICATION timer.
        button.Trigger(lambda: self.operator_controller_raw.get_button("MENU")).onTrue(AmpLEDs(self.leds))
        button.Trigger(lambda: self.operator_controller_raw.get_button("A")).toggleOnTrue(
            commands2.cmd.SequentialCommandGroup(
                commands2.cmd.runOnce(lambda: self.leds.reset_flames(), self.leds).ignoringDisable(True),
                commands2.cmd.run(lambda: self.leds.set_state("flames"), self.leds).ignoringDisable(True)
            ))
        button.Trigger(lambda: self.operator_controller_raw.get_button("B")).toggleOnTrue(
            commands2.cmd.run(lambda: self.leds.set_state("flash_color"), self.leds).ignoringDisable(True))
        button.Trigger(lambda: self.operator_controller_raw.get_button("X")).toggleOnTrue(
            commands2.cmd.run(lambda: self.leds.set_state("rainbow"), self.leds).ignoringDisable(True))

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
        NamedCommands.registerCommand("rainbow_leds", commands2.cmd.runOnce(lambda: self.leds.set_state("rainbow"),
                                                                            self.leds))
        NamedCommands.registerCommand("flash_green",
                                      commands2.SequentialCommandGroup(
                                          commands2.cmd.runOnce(lambda: self.leds.set_flash_color_color([255, 0, 0]),
                                                                self.leds),
                                          commands2.cmd.runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          commands2.cmd.runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_red",
                                      commands2.SequentialCommandGroup(
                                          commands2.cmd.runOnce(lambda: self.leds.set_flash_color_color([0, 255, 0]),
                                                                self.leds),
                                          commands2.cmd.runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          commands2.cmd.runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_blue",
                                      commands2.SequentialCommandGroup(
                                          commands2.cmd.runOnce(lambda: self.leds.set_flash_color_color([0, 0, 255]),
                                                                self.leds),
                                          commands2.cmd.runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          commands2.cmd.runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_purple",
                                      commands2.SequentialCommandGroup(
                                          commands2.cmd.runOnce(lambda: self.leds.set_flash_color_color([50, 149, 168]),
                                                                self.leds),
                                          commands2.cmd.runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          commands2.cmd.runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_yellow",
                                      commands2.SequentialCommandGroup(
                                          commands2.cmd.runOnce(lambda: self.leds.set_flash_color_color([255, 255, 0]),
                                                                self.leds),
                                          commands2.cmd.runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          commands2.cmd.runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("default_leds", commands2.cmd.runOnce(lambda: self.leds.set_state("default"),
                                                                            self.leds))
