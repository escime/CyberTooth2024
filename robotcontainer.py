import commands2
from commands2 import button

from constants import OIConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.ledsubsystem import LEDs
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from subsystems.tuningsubsystem import TuningSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager
from commands.default_leds import DefaultLEDs
from commands.debug_mode import DebugMode
from commands.shoot_leds import ShootLEDs
from commands.return_wheels import ReturnWheels
from commands.turn import Turn
from commands.amp_leds import AmpLEDs
from commands.flash_LL import FlashLL
from commands.ready_shooter import ReadyShooter
from commands.shoot import Shoot
from commands.shoot_vision import ShootVision
from commands.score_amp import ScoreAMP
from commands.drive_to_note import DriveToNote
from commands.intake import Intake
from commands.eject import Eject
from commands.ready_amp import ReadyAMP
from commands.switch_channel import SwitchPDHChannel
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
        tuning_setter = False
        # TODO Enable when you want to log data. Will Try enabling periodically.
        # DataLogManager.start()
        # DriverStation.startDataLog(DataLogManager.getLog(), True)

        # Instantiate subsystems using their constructors if tuning mode is disabled.
        if not tuning_setter:
            self.robot_drive = DriveSubsystem()
            self.leds = LEDs(0, 20, 1, 0.03, "GRB")
            self.vision_system = VisionSubsystem()
            self.utilsys = UtilSubsystem()  # Only compatible with REV PDH at this time.
            self.shooter = ShooterSubsystem()
            self.intake = IntakeSubsystem()
            self.trapper = TrapperSubsystem()

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(OIConstants.kDriverControllerPort, "xbox")
        self.operator_controller_raw = CustomHID(OIConstants.kOperatorControllerPort, "xbox")
        DriverStation.silenceJoystickConnectionWarning(True)

        # Perform setup as normal, unless tuning mode is enabled.
        if not tuning_setter:
            # Set the default drive command.
            self.robot_drive.setDefaultCommand(commands2.cmd.run(
                lambda: self.robot_drive.drive_2ok(
                    self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                    self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
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
                               "A_Score4", "B_Score4", "C_Score4", "A_Score2", "C_Score2"]
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
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("RX", 0.06) * DriveConstants.kMaxAngularSpeed,
                True,
                self.driver_controller_raw.refine_trigger("R", 0.05, 0.8, 0.3)), self.robot_drive))

        # Press any direction on the D-pad to enable PID snap to that equivalent angle based on field orientation
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("W")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                270
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("E")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                45
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("N")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                180
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("S")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                0
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_button("A")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                45
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_button("X")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                315
            ), self.robot_drive))

        # Reset robot pose to center of the field.
        button.Trigger(lambda: self.driver_controller_raw.get_button("Y")).whileTrue(
            commands2.cmd.run(lambda: self.vision_system.reset_hard_odo(self.robot_drive), self.vision_system,
                              self.robot_drive))

        # Emergency intake manual controls.
        button.Trigger(lambda: self.operator_controller_raw.get_button("RB")).whileTrue(
            commands2.cmd.run(lambda: self.intake.intake(1),
                              self.intake))
        button.Trigger(lambda: self.operator_controller_raw.get_button("RB")).onFalse(
            commands2.cmd.run(lambda: self.intake.intake(0), self.intake))
        button.Trigger(lambda: self.operator_controller_raw.get_button("LB")).whileTrue(
            commands2.cmd.run(lambda: self.intake.intake(-1),
                              self.intake))
        button.Trigger(lambda: self.operator_controller_raw.get_button("LB")).onFalse(
            commands2.cmd.run(lambda: self.intake.intake(0), self.intake))

        # Hold to manually shoot a NOTE.
        button.Trigger(lambda: self.driver_controller_raw.get_button("LB")).whileTrue(
            commands2.SequentialCommandGroup(
                ReadyShooter(self.shooter, "subwoofer"),
                Shoot("readied", True, self.shooter, self.intake, self.trapper)))

        # Hold for test shot (temporary lol)
        # button.Trigger(lambda: self.driver_controller_raw.get_button("RB")).whileTrue(
        #     commands2.SequentialCommandGroup(
        #         ReadyShooter(self.shooter, "test"),
        #         Shoot("readied", True, self.shooter, self.intake, self.trapper)))

        # Hold to autonomously shoot a NOTE.
        button.Trigger(lambda: self.driver_controller_raw.get_button("RB")).whileTrue(commands2.SequentialCommandGroup(
            ShootVision(True, self.shooter, self.vision_system, self.robot_drive, self.intake, self.trapper, self.leds),
            commands2.ParallelDeadlineGroup(
                Shoot("readied", True, self.shooter, self.intake, self.trapper),
                ShootLEDs(self.leds, "slow"))))

        # Press to prepare to place a NOTE in the AMP.
        button.Trigger(lambda: self.operator_controller_raw.get_button("A")).onTrue(
            ReadyAMP(self.trapper, self.shooter, self.robot_drive))

        # Hold to score a NOTE in the AMP. Release to return to STOW.
        button.Trigger(lambda: self.operator_controller_raw.get_button("X")).whileTrue(
            ScoreAMP(self.trapper, self.robot_drive))

        # Hold to drive towards and collect a NOTE.
        # button.Trigger(lambda: self.driver_controller_raw.get_button("B")).whileTrue(
        #     DriveToNote(self.robot_drive, self.intake, self.vision_system, self.trapper, self.leds))

        # When a NOTE enters the trapper, flash all LEDs green.
        button.Trigger(lambda: self.trapper.get_note_acquired()).onTrue(FlashLL(self.vision_system, self.leds))

        # Start an AMPLIFICATION timer.
        button.Trigger(lambda: self.operator_controller_raw.get_button("MENU")).onTrue(AmpLEDs(self.leds))

        # When the robot is enabled, turn on the Time-On Meter.
        button.Trigger(lambda: DriverStation.isEnabled()).onTrue(SwitchPDHChannel(True, self.utilsys))
        button.Trigger(lambda: DriverStation.isEnabled()).onFalse(SwitchPDHChannel(False, self.utilsys))

        # Manually control the arm.
        button.Trigger(lambda: self.operator_controller_raw.get_axis_triggered("RY", 0.1) and
                       self.shooter.angle_setpoint == self.shooter.angle_setpoints["stow"]).whileTrue(
            commands2.cmd.run(lambda: self.trapper.manual_arm(self.operator_controller_raw.get_axis("RY", 0.1) * -1),
                              self.trapper))
        button.Trigger(lambda: self.operator_controller_raw.get_axis_triggered("RY", 0.1)).onFalse(
            commands2.cmd.runOnce(lambda: self.trapper.manual_arm_off(), self.trapper))

        # Manually control the climber.
        button.Trigger(lambda: self.operator_controller_raw.get_axis_triggered("LY", 0.15)).whileTrue(
            commands2.cmd.run(lambda: self.trapper.run_climb(self.operator_controller_raw.get_axis("LY", 0.15) * -1),
                              self.trapper))
        button.Trigger(lambda: self.operator_controller_raw.get_axis_triggered("LY", 0.05)).onFalse(
            commands2.cmd.run(lambda: self.trapper.run_climb(0), self.trapper))

        # Manually control the trap intake.
        button.Trigger(lambda: self.operator_controller_raw.get_d_pad_pull("E")).whileTrue(
             commands2.cmd.run(lambda: self.trapper.manual_trap(0.5), self.trapper))
        button.Trigger(lambda: self.operator_controller_raw.get_d_pad_pull("W")).whileTrue(
            commands2.cmd.run(lambda: self.trapper.manual_trap(-0.5), self.trapper))
        button.Trigger(lambda: self.operator_controller_raw.get_d_pad_pull("E")).onFalse(
            commands2.cmd.run(lambda: self.trapper.manual_trap(0), self.trapper))
        button.Trigger(lambda: self.operator_controller_raw.get_d_pad_pull("W")).onFalse(
            commands2.cmd.run(lambda: self.trapper.manual_trap(0), self.trapper))

        # Adjust shooter trim.
        button.Trigger(lambda: self.operator_controller_raw.get_d_pad_pull("N")).onTrue(
            commands2.cmd.runOnce(lambda: self.shooter.increment_trim(0.05), self.shooter))
        button.Trigger(lambda: self.operator_controller_raw.get_d_pad_pull("S")).onTrue(
            commands2.cmd.runOnce(lambda: self.shooter.increment_trim(-0.05), self.shooter))

        # Hold to intake a NOTE.
        button.Trigger(lambda: self.operator_controller_raw.get_trigger("R", 0.1)).whileTrue(
            Intake(self.intake, self.trapper))

        # Hold to spit out a NOTE.
        button.Trigger(lambda: self.operator_controller_raw.get_trigger("L", 0.1)).whileTrue(
            Eject(self.intake, self.trapper, self.shooter))

        # Press to preset for climb approach.
        button.Trigger(lambda: self.operator_controller_raw.get_button("B")).onTrue(
            commands2.SequentialCommandGroup(
                ReadyShooter(self.shooter, "stow"),
                commands2.cmd.run(lambda: self.trapper.set_climb_stage_1(), self.trapper)))

        # Press to preset for climbing.
        button.Trigger(lambda: self.operator_controller_raw.get_button("Y")).onTrue(
            commands2.ParallelCommandGroup(
                commands2.cmd.run(lambda: self.trapper.set_climb_stage_2(), self.trapper),
                commands2.cmd.run(lambda: self.robot_drive.drive_2ok(0.05 * DriveConstants.kMaxSpeed, 0, 0, False),
                                  self.robot_drive)))

        # If climbing, set the leds to rainbow!
        button.Trigger(lambda: self.trapper.is_climbing).whileTrue(
            commands2.cmd.run(lambda: self.leds.rainbow_shift(), self.leds))

        # Enable odo in auto.
        button.Trigger(lambda: DriverStation.isAutonomous()).onTrue(ToggleOdo(self.vision_system))
        button.Trigger(lambda: DriverStation.isAutonomous() and self.vision_system.vision_odo).onFalse(
            ToggleOdo(self.vision_system))

        # Temporary controls setup for shooter tuning.
        # button.Trigger(lambda: self.driver_controller_raw.get_button("RB")).onTrue(
        #     commands2.cmd.runOnce(lambda: self.shooter.tuning_toggler(True), self.shooter))
        # button.Trigger(lambda: self.driver_controller_raw.get_button("RB")).onFalse(
        #     commands2.cmd.runOnce(lambda: self.shooter.tuning_toggler(False), self.shooter))
        button.Trigger(lambda: self.driver_controller_raw.get_button("VIEW")).onTrue(
            commands2.cmd.runOnce(lambda: self.shooter.set_known_setpoint("stow"), self.shooter))
        button.Trigger(lambda: self.driver_controller_raw.get_button("MENU")).onTrue(
            commands2.cmd.runOnce(lambda: self.shooter.set_known_setpoint("readied"), self.shooter))
        # button.Trigger(lambda: self.driver_controller_raw.get_button("MENU")).onTrue(
        #     commands2.cmd.runOnce(lambda: self.shooter.set_known_setpoint("subwoofer"), self.shooter))

        # button.Trigger(lambda: self.operator_controller_raw.get_button("X")).onTrue(ShootLEDs(self.leds, "slow"))

        # Vibrate the driver controller when targets are in view (update later to be "within vision table")
        button.Trigger(lambda: self.vision_system.range_to_angle() != -1).whileTrue(
            commands2.cmd.run(lambda: self.driver_controller_raw.set_rumble(1)))
        button.Trigger(lambda: self.vision_system.range_to_angle() != -1).whileFalse(
            commands2.cmd.run(lambda: self.driver_controller_raw.set_rumble(0)))

    def configureTriggers1ControllerProfile(self):
        # Reset robot pose to center of the field.
        button.Trigger(lambda: self.driver_controller_raw.get_button("Y")).whileTrue(
            commands2.cmd.run(lambda: self.vision_system.reset_hard_odo(self.robot_drive), self.vision_system,
                              self.robot_drive))

        # Hold to manually shoot a NOTE.
        button.Trigger(lambda: self.driver_controller_raw.get_button("LB")).whileTrue(
            commands2.SequentialCommandGroup(
                ReadyShooter(self.shooter, "subwoofer"),
                Shoot("readied", True, self.shooter, self.intake, self.trapper)))

        # Hold to autonomously shoot a NOTE.
        button.Trigger(lambda: self.driver_controller_raw.get_button("RB")).whileTrue(commands2.SequentialCommandGroup(
            ShootVision(True, self.shooter, self.vision_system, self.robot_drive, self.intake, self.trapper, self.leds),
            commands2.ParallelDeadlineGroup(
                Shoot("readied", True, self.shooter, self.intake, self.trapper),
                ShootLEDs(self.leds, "slow"))))

        # Press to prepare to place a NOTE in the AMP.
        button.Trigger(lambda: self.driver_controller_raw.get_button("A")).onTrue(
            ReadyAMP(self.trapper, self.shooter, self.robot_drive))

        # Hold to score a NOTE in the AMP. Release to return to STOW.
        button.Trigger(lambda: self.driver_controller_raw.get_button("X")).whileTrue(
            ScoreAMP(self.trapper, self.robot_drive))

        # When a NOTE enters the trapper, flash all LEDs green.
        button.Trigger(lambda: self.trapper.get_note_acquired()).onTrue(FlashLL(self.vision_system, self.leds))

        # When the robot is enabled, turn on the Time-On Meter.
        button.Trigger(lambda: DriverStation.isEnabled()).onTrue(SwitchPDHChannel(True, self.utilsys))
        button.Trigger(lambda: DriverStation.isEnabled()).onFalse(SwitchPDHChannel(False, self.utilsys))

        # Manually control the climber.
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("S")).whileTrue(
            commands2.cmd.run(lambda: self.trapper.run_climb(1), self.trapper))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("S")).onFalse(
            commands2.cmd.run(lambda: self.trapper.run_climb(0), self.trapper))

        # Manually control the trap intake.
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("E")).whileTrue(
            commands2.cmd.run(lambda: self.trapper.manual_trap(0.5), self.trapper))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("W")).whileTrue(
            commands2.cmd.run(lambda: self.trapper.manual_trap(-0.5), self.trapper))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("E")).onFalse(
            commands2.cmd.run(lambda: self.trapper.manual_trap(0), self.trapper))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("W")).onFalse(
            commands2.cmd.run(lambda: self.trapper.manual_trap(0), self.trapper))

        # Hold to intake a NOTE.
        button.Trigger(lambda: self.driver_controller_raw.get_trigger("R", 0.1)).whileTrue(
            Intake(self.intake, self.trapper))

        # Hold to spit out a NOTE.
        button.Trigger(lambda: self.driver_controller_raw.get_trigger("L", 0.1)).whileTrue(
            Eject(self.intake, self.trapper, self.shooter))

        # Press to preset for climb approach.
        button.Trigger(lambda: self.driver_controller_raw.get_button("VIEW")).onTrue(
            commands2.SequentialCommandGroup(
                ReadyShooter(self.shooter, "stow"),
                commands2.cmd.run(lambda: self.trapper.set_climb_stage_1(), self.trapper)))

        # Press to preset for climbing.
        button.Trigger(lambda: self.driver_controller_raw.get_button("MENU")).onTrue(
            commands2.ParallelCommandGroup(
                commands2.cmd.run(lambda: self.trapper.set_climb_stage_2(), self.trapper),
                commands2.cmd.run(lambda: self.robot_drive.drive_2ok(0.05 * DriveConstants.kMaxSpeed, 0, 0, False),
                                  self.robot_drive)))

        # If climbing, set the leds to rainbow!
        button.Trigger(lambda: self.trapper.is_climbing).whileTrue(
            commands2.cmd.run(lambda: self.leds.rainbow_shift(), self.leds))

        # Enable odo in auto. Maybe this works as planned?
        button.Trigger(lambda: DriverStation.isAutonomous()).onTrue(ToggleOdo(self.vision_system))
        button.Trigger(lambda: DriverStation.isAutonomous() and self.vision_system.vision_odo).onFalse(
            ToggleOdo(self.vision_system))

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
        NamedCommands.registerCommand("ready_shooter", ReadyShooter(self.shooter, "subwoofer"))
        NamedCommands.registerCommand("intake", Intake(self.intake, self.trapper))
        NamedCommands.registerCommand("drive_to_note", DriveToNote(self.robot_drive, self.intake,
                                                                   self.vision_system, self.trapper, self.leds))
        NamedCommands.registerCommand("flash_LL", FlashLL(self.vision_system, self.leds))
        NamedCommands.registerCommand("shoot_vision", commands2.SequentialCommandGroup(
            ShootVision(False, self.shooter, self.vision_system, self.robot_drive, self.intake,
                        self.trapper, self.leds),
            commands2.ParallelDeadlineGroup(
                Shoot("readied", False, self.shooter, self.intake, self.trapper),
                ShootLEDs(self.leds, "slow"))))
        NamedCommands.registerCommand("shoot", commands2.ParallelDeadlineGroup(
                Shoot("readied", False, self.shooter, self.intake, self.trapper),
                ShootLEDs(self.leds, "slow")))
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
