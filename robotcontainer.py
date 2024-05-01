import commands2
import wpilib.simulation
from commands2 import button
from constants import OIConstants, DriveConstants
from subsystems.phoenixdrivesubsystem import PhoenixDriveSubsystem
from subsystems.ledsubsystem import LEDs
from subsystems.visionsubsystem import VisionSubsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager, LiveWindow, Timer
from commands.default_leds import DefaultLEDs
from commands.debug_mode import DebugMode
from commands.return_wheels import ReturnWheels
from commands.turn import Turn
from commands.flash_LL import FlashLL
from commands.toggle_odo import ToggleOdo
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

        LiveWindow.disableAllTelemetry()

        self.drive = PhoenixDriveSubsystem(self.timer)
        self.leds = LEDs(0, 20, 1, 0.03, "GRB", self.timer)
        self.vision_system = VisionSubsystem(self.timer, self.drive)

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(OIConstants.kDriverControllerPort, "xbox")
        self.operator_controller_raw = CustomHID(OIConstants.kOperatorControllerPort, "xbox")
        DriverStation.silenceJoystickConnectionWarning(True)

        self.drive.setDefaultCommand(commands2.cmd.run(
            lambda: self.drive.drive_closed_loop_turning(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed * 0.9,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed * 0.9,
                self.driver_controller_raw.get_axis("RX", 0.06) * -1,
                7,
                0.2
            ), self.drive
        ))

        # Set default subsystem commands.
        self.leds.setDefaultCommand(DefaultLEDs(self.leds))

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
                           "C_Score3_Midline", "Chaos", "B_Score4.5"]
        self.m_chooser.setDefaultOption("DoNothing", "DoNothing")
        for x in self.auto_names:
            self.m_chooser.addOption(x, x)

        SmartDashboard.putData("Auto Select", self.m_chooser)

        SmartDashboard.putData("Debug Mode On", DebugMode(self.drive, True))
        SmartDashboard.putData("Debug Mode Off", DebugMode(self.drive, False))

    def configureTriggersDefault(self) -> None:
        """Used to set up any commands that trigger when a measured event occurs."""
        # Hold for Parking Brake.
        # button.Trigger(lambda: self.driver_controller_raw.get_trigger("L", 0.05)).whileTrue(
        #     commands2.cmd.run(lambda: self.robot_drive.drive_lock(), self.robot_drive))

        # Hold for Slow Mode, variable based on depth of Trigger.
        button.Trigger(lambda: self.driver_controller_raw.get_trigger("R", 0.05)).whileTrue(
            commands2.cmd.run(lambda: self.drive.drive_slow_clt(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("RX", 0.06) * DriveConstants.kMaxAngularSpeed,
                True,
                self.driver_controller_raw.refine_trigger("R", 0.05, 0.8, 0.3)), self.drive))

        # Press any direction on the D-pad to enable PID snap to that equivalent angle based on field orientation
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("W")).toggleOnTrue(
            commands2.cmd.run(lambda: self.drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                270
            ), self.drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("E")).toggleOnTrue(
            commands2.cmd.run(lambda: self.drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                90
            ), self.drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("N")).toggleOnTrue(
            commands2.cmd.run(lambda: self.drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                180
            ), self.drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("S")).toggleOnTrue(
            commands2.cmd.run(lambda: self.drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                0
            ), self.drive))
        button.Trigger(lambda: self.driver_controller_raw.get_button("A")).toggleOnTrue(
            commands2.cmd.run(lambda: self.drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                60
            ), self.drive))
        button.Trigger(lambda: self.driver_controller_raw.get_button("X")).toggleOnTrue(
            commands2.cmd.run(lambda: self.drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                300
            ), self.drive))

        # When pressing the underside triggers, automatically drive the robot to the correct position under the stage
        # button.Trigger(lambda: self.driver_controller_raw.get_button("X")).whileTrue(
        #     self.robot_drive.follow_path_command([4.68, 3.76, -120], -120))
        # button.Trigger(lambda: self.driver_controller_raw.get_button("A")).whileTrue(
        #     self.robot_drive.follow_path_command([4.7, 4.3, 120], 120))

        # Reset robot pose to center of the field.
        button.Trigger(lambda: self.driver_controller_raw.get_button("Y")).whileTrue(
            commands2.cmd.run(lambda: self.vision_system.reset_hard_odo(self.drive), self.vision_system,
                              self.drive))

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
        NamedCommands.registerCommand("return_wheels", ReturnWheels(self.drive))
        NamedCommands.registerCommand("flash_LL", FlashLL(self.vision_system, self.leds))
        NamedCommands.registerCommand("turn_north", Turn(self.drive, 0, self.timer))
        NamedCommands.registerCommand("toggle_odo", ToggleOdo(self.vision_system))
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
