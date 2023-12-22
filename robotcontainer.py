import commands2
import commands2.cmd

from constants import OIConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.ledsubsystem import LEDs
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from subsystems.tuningsubsystem import TuningSubsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager
from commands.default_leds import DefaultLEDs
from commands.debug_mode import DebugMode
from commands.return_wheels import ReturnWheels
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
        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog(), True)
        # Instantiate subsystems using their constructors.
        if not tuning_setter:
            self.robot_drive = DriveSubsystem()
            self.leds = LEDs(0, 30, 1, 0.03, "GRB")  # TODO CHANGE TO CORRECT LED COUNT
            self.vision_system = VisionSubsystem(self.robot_drive)
            self.utilsys = UtilSubsystem()  # Only compatible with REV PDH at this time.

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(OIConstants.kDriverControllerPort, "xbox")
        self.operator_controller_raw = CustomHID(OIConstants.kOperatorControllerPort, "xbox")
        DriverStation.silenceJoystickConnectionWarning(True)

        if not tuning_setter:
            # Set the default drive command.
            self.robot_drive.setDefaultCommand(commands2.cmd.run(
                lambda: self.robot_drive.drive_2ok(
                    self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                    self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                    self.driver_controller_raw.get_axis("RX", 0.06) *
                    DriveConstants.kMaxAngularSpeed,
                    True, ),
                self.robot_drive
            ))

            # Set default LED command.
            self.leds.setDefaultCommand(DefaultLEDs(self.leds))

            # Register commands for PathPlanner.
            self.registerCommands()

            # Setup for all event-trigger commands.
            self.configureTriggersDefault()

            # Setup autonomous selector on the dashboard.
            self.m_chooser = SendableChooser()
            self.m_chooser.setDefaultOption("No-op", "N-op")
            self.m_chooser.addOption("Test", "Test")

            SmartDashboard.putData("Auto Select", self.m_chooser)

            SmartDashboard.putData("Debug Mode On", DebugMode(self.robot_drive, True))
            SmartDashboard.putData("Debug Mode Off", DebugMode(self.robot_drive, False))

        else:
            self.tuner = TuningSubsystem(True, True, 50)
            self.configureTuningMode()

    def configureTuningMode(self) -> None:
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("A")).onTrue(
            commands2.cmd.run(lambda: self.tuner.id_ks_dt(0.1), self.tuner)
        )
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("B")).onTrue(
            commands2.cmd.runOnce(lambda: self.tuner.id_kv_dt(4), self.tuner)
        )
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("X")).onTrue(
            commands2.cmd.runOnce(lambda: self.tuner.set_all_zero(), self.tuner)
        )

    def configureTriggersDefault(self) -> None:
        """Used to set up any commands that trigger when a measured event occurs."""
        # Parking Brake.
        commands2.Trigger(lambda: self.driver_controller_raw.get_trigger("L", 0.05)).whileTrue(
            commands2.cmd.run(lambda: self.robot_drive.drive_lock(), self.robot_drive))

        # Slow mode.
        commands2.Trigger(lambda: self.driver_controller_raw.get_trigger("R", 0.05)).whileTrue(
            commands2.cmd.run(lambda: self.robot_drive.drive_slow(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("RX", 0.06) * DriveConstants.kMaxAngularSpeed,
                True,
                self.driver_controller_raw.refine_trigger("R", 0.05, 0.9, 0.1)), self.robot_drive))

        # Press any direction on the D-pad to enable PID snap to that equivalent angle based on field orientation
        commands2.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("W")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                -90
            ), self.robot_drive))
        commands2.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("E")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                90
            ), self.robot_drive))
        commands2.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("N")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                0
            ), self.robot_drive))
        commands2.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("S")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                180
            ), self.robot_drive))

        # Reset robot pose to center of the field.
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("Y")).whileTrue(
            commands2.cmd.run(lambda: self.vision_system.reset_hard_odo(), self.vision_system, self.robot_drive))

    def getAutonomousCommand(self) -> commands2.cmd:
        """Use this to pass the autonomous command to the main Robot class.
        Returns the command to run in autonomous
        """
        if self.m_chooser.getSelected() == "No-op":
            return None
        elif self.m_chooser.getSelected() == "Test":
            return PathPlannerAuto("Test")
        else:
            return None

    def registerCommands(self):
        NamedCommands.registerCommand("return_wheels", ReturnWheels(self.robot_drive))
