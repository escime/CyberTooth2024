import commands2
from commands2 import button

from constants import OIConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.ledsubsystem import LEDs
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.tuningsubsystem import TuningSubsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager
from commands.default_leds import DefaultLEDs
from commands.debug_mode import DebugMode
from commands.return_wheels import ReturnWheels
from commands.turn import Turn
from commands.flash_LL import FlashLL
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
        # TODO Enable when you want to log data. Will Try enabling periodically.
        # DataLogManager.start()
        # DriverStation.startDataLog(DataLogManager.getLog(), True)

        # Instantiate subsystems using their constructors if tuning mode is disabled.
        if not tuning_setter:
            self.robot_drive = DriveSubsystem()
            self.leds = LEDs(0, 30, 1, 0.03, "GRB")  # TODO CHANGE TO CORRECT LED COUNT
            self.vision_system = VisionSubsystem(self.robot_drive)

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(OIConstants.kDriverControllerPort, "xbox")
        self.operator_controller_raw = CustomHID(OIConstants.kOperatorControllerPort, "xbox")
        DriverStation.silenceJoystickConnectionWarning(True)

        # Perform setup as normal, unless tuning mode is enabled.
        if not tuning_setter:
            # Set the default drive command.
            self.robot_drive.setDefaultCommand(commands2.cmd.run(
                lambda: self.robot_drive.drive_2ok(
                    self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                    self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                    self.driver_controller_raw.get_axis("RX", 0.06) *
                    DriveConstants.kMaxAngularSpeed,
                    True),
                self.robot_drive
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
                               "A_Score4", "B_Score4", "C_Score4", "A_Score2", "C_Score2", "A_Score2_Fast",
                               "C_Score2_Fast"]
            self.m_chooser.setDefaultOption("DoNothing", "DoNothing")
            for x in self.auto_names:
                self.m_chooser.addOption(x, x)

            SmartDashboard.putData("Auto Select", self.m_chooser)

            SmartDashboard.putData("Debug Mode On", DebugMode(self.robot_drive, True))
            SmartDashboard.putData("Debug Mode Off", DebugMode(self.robot_drive, False))

        # Perform setup for when tuning mode is enabled.
        else:
            self.tuner = TuningSubsystem(True, True, 50)
            self.configureTuningMode()

    def configureTuningMode(self) -> None:

        button.Trigger(lambda: self.driver_controller_raw.get_button("A")).onTrue(
            commands2.cmd.run(lambda: self.tuner.id_ks_dt(0.0001), self.tuner)
        )
        button.Trigger(lambda: self.driver_controller_raw.get_button("B")).onTrue(
            commands2.cmd.runOnce(lambda: self.tuner.id_kv_dt(6), self.tuner)
        )
        button.Trigger(lambda: self.driver_controller_raw.get_button("X")).onTrue(
            commands2.cmd.runOnce(lambda: self.tuner.set_all_zero(), self.tuner)
        )
        button.Trigger(lambda: self.driver_controller_raw.get_button("Y")).onTrue(
            commands2.cmd.runOnce(lambda: self.tuner.reset_routines(), self.tuner)
        )

    def configureTriggersDefault(self) -> None:
        """Used to set up any commands that trigger when a measured event occurs."""
        # Hold for Parking Brake.
        button.Trigger(lambda: self.driver_controller_raw.get_trigger("L", 0.05)).whileTrue(
            commands2.cmd.run(lambda: self.robot_drive.drive_lock(), self.robot_drive))

        # Hold for Slow Mode, variable based on depth of Trigger.
        button.Trigger(lambda: self.driver_controller_raw.get_trigger("R", 0.05)).whileTrue(
            commands2.cmd.run(lambda: self.robot_drive.drive_slow(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("RX", 0.06) * DriveConstants.kMaxAngularSpeed,
                True,
                self.driver_controller_raw.refine_trigger("R", 0.05, 0.9, 0.3)), self.robot_drive))

        # Press any direction on the D-pad to enable PID snap to that equivalent angle based on field orientation
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("W")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                270
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("E")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                90
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("N")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                180
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("S")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                0
            ), self.robot_drive))

        # Reset robot pose to center of the field.
        button.Trigger(lambda: self.driver_controller_raw.get_button("Y")).whileTrue(
            commands2.cmd.run(lambda: self.vision_system.reset_hard_odo(), self.vision_system, self.robot_drive))

        # Enable odo in auto.
        button.Trigger(lambda: self.driver_controller_raw.get_button("LB")).onTrue(
            commands2.cmd.runOnce(lambda: self.vision_system.vision_odo_toggle()))

        button.Trigger(lambda: self.driver_controller_raw.get_button("RB")).toggleOnTrue(
            commands2.cmd.run(lambda: self.vision_system.rotate_to_target(self.robot_drive,
                                                                          self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                                                                          self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                                                                          ), self.vision_system, self.robot_drive)
        )

        button.Trigger(lambda: self.vision_system.target_locked).onTrue(
            FlashLL(self.vision_system, self.leds))

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
        NamedCommands.registerCommand("return_wheels", ReturnWheels(self.robot_drive))
        NamedCommands.registerCommand("turn_north", Turn(self.robot_drive, 0))
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
