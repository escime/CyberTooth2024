import commands2
import wpilib.simulation
from commands2 import button

from constants import OIConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
# from subsystems.ledsubsystem import LEDs
from subsystems.ledsubsystem2 import LEDs
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from subsystems.tuningsubsystem import TuningSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager, LiveWindow, Timer
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
from commands.shoot_vision_mod import ShootVisionMod
from commands.score_amp import ScoreAMP
# from commands.drive_to_note import DriveToNote
from commands.intake import Intake
from commands.eject import Eject
from commands.ready_amp import ReadyAMP
from commands.switch_channel import SwitchPDHChannel
from commands.toggle_odo import ToggleOdo
from commands.vision_estimate import VisionEstimate
from commands.alert_gp_leds import AlertGPLEDs
from commands.climb_s1 import ClimbS1
from commands.climb_s2 import ClimbS2
from commands.maintain_shooter import MaintainShooter
from commands.shoot_while_moving import ShootVisionWhileMoving
from commands.shoot_vision_feed import ShootVisionFeed
from commands.shoot_odo_only import ShootVisionOdo
from helpers.custom_hid import CustomHID
from pathplannerlib.auto import NamedCommands, PathPlannerAuto
from commands.passthrough import Passthrough
from commands.check_drivetrain import CheckDrive
from commands.check_shooter import CheckShooter
from commands.check_arm import CheckArm
from commands.check_intake import CheckIntake
from commands.master_caution import MasterCaution
from helpers.pose_estimator import PoseEstimator


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

        # Instantiate subsystems using their constructors if tuning mode is disabled.
        if not tuning_setter:
            self.pose_estimator = PoseEstimator()
            self.robot_drive = DriveSubsystem(self.timer, self.pose_estimator)
            # self.leds = LEDs(0, 20, 1, 0.03, "GRB", self.timer)
            self.leds = LEDs(self.timer)
            self.trapper = TrapperSubsystem()
            self.vision_system = VisionSubsystem(self.timer, self.pose_estimator, self.trapper)
            self.utilsys = UtilSubsystem()  # Only compatible with REV PDH at this time.
            self.shooter = ShooterSubsystem()
            self.intake = IntakeSubsystem()

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(OIConstants.kDriverControllerPort, "xbox")
        self.operator_controller_raw = CustomHID(OIConstants.kOperatorControllerPort, "xbox")
        DriverStation.silenceJoystickConnectionWarning(True)

        # Perform setup as normal, unless tuning mode is enabled.
        if not tuning_setter:
            self.robot_drive.setDefaultCommand(commands2.cmd.run(
                lambda: self.robot_drive.drive_2ok(
                    self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                    self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                    self.driver_controller_raw.get_axis_squared("RX", 0.06) * DriveConstants.kMaxAngularSpeed,
                    True
                ), self.robot_drive
            ))

            # self.robot_drive.setDefaultCommand(commands2.cmd.run(
            #     lambda: self.robot_drive.snap_drive(
            #         self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
            #         self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
            #         self.driver_controller_raw.dir_est_ctrl("R")
            #     ), self.robot_drive
            # ))

            # Set default subsystem commands.
            # self.leds.setDefaultCommand(DefaultLEDs(self.leds))

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
                               "C_Score3_Midline", "Chaos", "B_Score4.5", "A_Score3_Midline_Alt", "0SystemsCheck",
                               "A_Score5_Alt"]
            self.m_chooser.setDefaultOption("DoNothing", "DoNothing")
            for x in self.auto_names:
                self.m_chooser.addOption(x, x)

            SmartDashboard.putData("Auto Select", self.m_chooser)

            SmartDashboard.putData("Debug Mode On", DebugMode(self.robot_drive, True))
            SmartDashboard.putData("Debug Mode Off", DebugMode(self.robot_drive, False))

            SmartDashboard.putData("Shooter Subsystem", self.shooter)
            SmartDashboard.putData("Trapper Subsystem", self.trapper)
            SmartDashboard.putData("Drive Subsystem", self.robot_drive)

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
        button.Trigger(lambda: DriverStation.isTeleopEnabled()).onTrue(
            commands2.cmd.runOnce(lambda: self.robot_drive.set_alliance(), self.robot_drive))
        button.Trigger(lambda: DriverStation.isDSAttached()).onTrue(
            commands2.cmd.runOnce(lambda: self.robot_drive.set_alliance(), self.robot_drive))

        # Hold for Parking Brake.
        # button.Trigger(lambda: self.driver_controller_raw.get_trigger("L", 0.05)).whileTrue(
        #     commands2.cmd.run(lambda: self.robot_drive.drive_lock(), self.robot_drive))

        button.Trigger(lambda: self.driver_controller_raw.get_button("RTHUMB")).whileTrue(
                ShootVisionWhileMoving(self.shooter, self.vision_system, self.intake, self.trapper,
                                       self.robot_drive, self.timer, self.driver_controller_raw, 10, 0.4))

        button.Trigger(lambda: self.driver_controller_raw.get_trigger("L", 0.3)).whileTrue(
            ShootVisionFeed(self.shooter, self.robot_drive, self.intake, self.trapper, self.vision_system,
                            self.driver_controller_raw))

        # Hold for Slow Mode, variable based on depth of Trigger.
        button.Trigger(lambda: self.driver_controller_raw.get_trigger("R", 0.05)).whileTrue(
            commands2.cmd.run(lambda: self.robot_drive.drive_slow(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("RX", 0.06),
                True,
                self.driver_controller_raw.refine_trigger("R", 0.05, 0.8, 0.3)), self.robot_drive))

        # Press any direction on the D-pad to enable PID snap to that equivalent angle based on field orientation
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
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("E")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                60
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("W")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                300
            ), self.robot_drive))

        # When pressing the underside triggers, automatically drive the robot to the correct position under the stage
        # button.Trigger(lambda: self.driver_controller_raw.get_button("X")).whileTrue(
        #     self.robot_drive.follow_path_command([4.68, 3.76, -120], -120))
        # button.Trigger(lambda: self.driver_controller_raw.get_button("A")).whileTrue(
        #     self.robot_drive.follow_path_command([4.7, 4.3, 120], 120))

        # Reset robot pose to subwoofer.
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
                ReadyShooter(self.shooter, "subwoofer", self.timer),
                Shoot("readied", True, self.shooter, self.intake, self.trapper, self.timer)))

        # Hold for test shot (temporary lol)
        # button.Trigger(lambda: self.driver_controller_raw.get_button("RB")).whileTrue(
        #     commands2.SequentialCommandGroup(
        #         ReadyShooter(self.shooter, "test", self.timer),
        #         Shoot("readied", True, self.shooter, self.intake, self.trapper, self.timer)))

        # Hold to autonomously shoot a NOTE.
        button.Trigger(lambda: self.driver_controller_raw.get_button("RB")).whileTrue(commands2.SequentialCommandGroup(
            ShootVision(True, self.shooter, self.vision_system, self.robot_drive, self.intake, self.trapper,
                        self.leds, self.timer),
            commands2.ParallelCommandGroup(
               commands2.cmd.run(lambda: self.robot_drive.drive(0, 0, 0, False), self.robot_drive),
               Shoot("readied", True, self.shooter, self.intake, self.trapper, self.timer),
               ShootLEDs(self.leds))))

        # Press to prepare to place a NOTE in the AMP.
        button.Trigger(lambda: self.operator_controller_raw.get_button("A")).toggleOnTrue( # or
                       # self.driver_controller_raw.get_button("A")).toggleOnTrue(
                       ReadyAMP(self.trapper, self.shooter, self.robot_drive, self.intake))

        # When A is pressed, generate a path to the AMP and, uh, see what happens
#         button.Trigger(lambda: self.driver_controller_raw.get_button("A") and
#                        DriverStation.getAlliance() == DriverStation.Alliance.kBlue).onTrue(
#             self.robot_drive.pathfind([1.91, 7.32, -90])
#         )
#         button.Trigger(lambda: self.driver_controller_raw.get_button("A") and not
#                        DriverStation.getAlliance() == DriverStation.Alliance.kBlue).onTrue(
#             self.robot_drive.pathfind([16.54 - 1.91, 7.32, -90])
#         )

        # Hold to score a NOTE in the AMP. Release to return to STOW.
        button.Trigger(lambda: self.operator_controller_raw.get_button("X")).onTrue(  # or
                       # self.driver_controller_raw.get_button("X")).onTrue(
                       ScoreAMP(self.trapper, self.robot_drive))

        # Press to toggle between auto shooting and manual shooting from the podium
        button.Trigger(lambda: self.driver_controller_raw.get_button("B")).onTrue(
            commands2.cmd.runOnce(lambda: self.vision_system.toggle_vision_shot_bypass(), self.vision_system))

        # When a NOTE enters the trapper, flash all LEDs green.
        button.Trigger(lambda: self.trapper.get_note_acquired() and
                       (DriverStation.isTeleopEnabled() or
                       DriverStation.isDisabled())).onTrue(commands2.SequentialCommandGroup(
                        FlashLL(self.vision_system, self.leds, self.timer),
                        AlertGPLEDs(self.leds, self.trapper)))

        # Start an AMPLIFICATION timer.
        button.Trigger(lambda: self.operator_controller_raw.get_button("MENU")).onTrue(AmpLEDs(self.leds))

        # When the robot is enabled, turn on the Time-On Meter.
        button.Trigger(lambda: DriverStation.isEnabled()).onTrue(SwitchPDHChannel(True, self.utilsys))
        button.Trigger(lambda: DriverStation.isEnabled()).onFalse(SwitchPDHChannel(False, self.utilsys))

        # Manually control the arm.
        button.Trigger(lambda: self.operator_controller_raw.get_axis_triggered("RY", 0.1)).whileTrue(
            commands2.cmd.SequentialCommandGroup(
                commands2.cmd.runOnce(lambda: self.shooter.set_known_setpoint("stow"), self.shooter),
                commands2.cmd.run(lambda: self.trapper.manual_arm(self.operator_controller_raw.get_axis("RY", 0.1) * -1),
                                  self.trapper)))
        button.Trigger(lambda: self.operator_controller_raw.get_axis_triggered("RY", 0.1)).onFalse(
            commands2.cmd.runOnce(lambda: self.trapper.manual_arm_off(), self.trapper))

        # Manually control the climber.
        button.Trigger(lambda: self.operator_controller_raw.get_axis_triggered("LY", 0.15)).whileTrue(
            commands2.cmd.run(lambda: self.trapper.run_climb(self.operator_controller_raw.get_axis("LY", 0.15) * -1),
                              self.trapper))
        button.Trigger(lambda: self.operator_controller_raw.get_axis_triggered("LY", 0.05)).onFalse(
            commands2.cmd.run(lambda: self.trapper.run_climb(0), self.trapper))
        button.Trigger(lambda: DriverStation.isDisabled()).onTrue(
            commands2.cmd.run(lambda: self.trapper.run_climb(0), self.trapper).ignoringDisable(True))

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
            commands2.cmd.runOnce(lambda: self.shooter.increment_trim(0.001), self.shooter))
        button.Trigger(lambda: self.operator_controller_raw.get_d_pad_pull("S")).onTrue(
            commands2.cmd.runOnce(lambda: self.shooter.increment_trim(-0.001), self.shooter))

        # Hold to intake a NOTE.
        button.Trigger(lambda: self.operator_controller_raw.get_trigger("R", 0.1)).whileTrue(  # or
                       # self.driver_controller_raw.get_button("B")).whileTrue(
                       Intake(self.intake, self.trapper, True, self.timer))

        # Hold to spit out a NOTE.
        button.Trigger(lambda: self.operator_controller_raw.get_trigger("L", 0.1)).whileTrue(
            Eject(self.intake, self.trapper, self.shooter))

        # Press to preset for climb approach.
        button.Trigger(lambda: self.operator_controller_raw.get_button("B")).onTrue(
            commands2.SequentialCommandGroup(
                ReadyShooter(self.shooter, "stow", self.timer),
                ClimbS1(self.trapper, self.leds, self.intake)))

        # Press to preset for climbing.
        button.Trigger(lambda: self.operator_controller_raw.get_button("Y")).onTrue(
             ClimbS2(self.trapper, self.leds, self.robot_drive))

        # If climbing, set the leds to rainbow!
        button.Trigger(lambda: self.trapper.is_climbing).whileTrue(
         commands2.cmd.runOnce(lambda: self.leds.set_state("rainbow"), self.leds).ignoringDisable(True))

        # Set shooter to STOW when pressing MENU on the driver controller
        button.Trigger(lambda: self.driver_controller_raw.get_button("MENU")).onTrue(
            commands2.cmd.runOnce(lambda: self.shooter.set_known_setpoint("stow"), self.shooter))
        # Set shooter to maintain angle while in targeting range
        button.Trigger(lambda: self.driver_controller_raw.get_button("VIEW")).toggleOnTrue(
            MaintainShooter(self.shooter, self.robot_drive, self.vision_system))

        # Vibrate the driver controller when targets are in view
        button.Trigger(lambda: self.vision_system.range_to_angle() != -1).whileTrue(
            commands2.cmd.run(lambda: self.driver_controller_raw.set_rumble(1)))
        button.Trigger(lambda: self.vision_system.range_to_angle() != -1).whileFalse(
            commands2.cmd.run(lambda: self.driver_controller_raw.set_rumble(0)).ignoringDisable(True))
        button.Trigger(lambda: DriverStation.isDisabled()).onTrue(
            commands2.cmd.runOnce(lambda: self.driver_controller_raw.set_rumble(0)).ignoringDisable(True))

        button.Trigger(lambda: self.operator_controller_raw.get_button("VIEW")).onTrue(
            commands2.SequentialCommandGroup(
                commands2.cmd.runOnce(lambda: self.trapper.reset_climber_zero(), self.trapper).ignoringDisable(True),
                commands2.cmd.runOnce(lambda: self.trapper.stop_climbing(), self.trapper).ignoringDisable(True))
            )

        # button.Trigger(lambda: self.driver_controller_raw.get_button("RTHUMB")).whileTrue(
        #     commands2.SequentialCommandGroup(
        #         commands2.ParallelDeadlineGroup(
        #             DriveToNote(self.robot_drive, self.intake, self.vision_system, self.trapper, self.timer),
        #             commands2.cmd.run(lambda: self.leds.flash_color([119, 247, 30], 2), self.leds)),
        #         commands2.cmd.run(lambda: self.trapper.manual_trap(1), self.trapper).withTimeout(0.25),
        #         commands2.cmd.runOnce(lambda: self.trapper.manual_trap(0), self.trapper))
        #     )

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
        NamedCommands.registerCommand("ready_shooter", ReadyShooter(self.shooter, "subwoofer", self.timer))
        NamedCommands.registerCommand("intake", Intake(self.intake, self.trapper, False, self.timer))
        # NamedCommands.registerCommand("drive_to_note", DriveToNote(self.robot_drive, self.intake,
        #                                                            self.vision_system, self.trapper, self.leds,
        #                                                            self.timer))
        NamedCommands.registerCommand("flash_LL", FlashLL(self.vision_system, self.leds, self.timer))
        NamedCommands.registerCommand("shoot_vision", commands2.SequentialCommandGroup(
            ShootVision(False, self.shooter, self.vision_system, self.robot_drive, self.intake,
                        self.trapper, self.leds, self.timer),
            commands2.ParallelDeadlineGroup(
                Shoot("readied", False, self.shooter, self.intake, self.trapper, self.timer),
                commands2.cmd.run(lambda: self.robot_drive.drive(0, 0, 0, False), self.robot_drive),
                ShootLEDs(self.leds))))
        NamedCommands.registerCommand("shoot", commands2.ParallelDeadlineGroup(
                Shoot("readied", False, self.shooter, self.intake, self.trapper, self.timer),
                ShootLEDs(self.leds)))
        NamedCommands.registerCommand("turn_north", Turn(self.robot_drive, 0, self.timer))
        NamedCommands.registerCommand("toggle_odo", ToggleOdo(self.vision_system))
        NamedCommands.registerCommand("vision_estimate", VisionEstimate(self.vision_system, self.robot_drive))
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
        NamedCommands.registerCommand("maintain_shooter", MaintainShooter(self.shooter, self.robot_drive,
                                                                          self.vision_system))
        NamedCommands.registerCommand("passthrough", Passthrough("readied", self.shooter, self.intake, self.trapper))
        NamedCommands.registerCommand("check_drive", CheckDrive(self.robot_drive, self.utilsys, self.timer))
        NamedCommands.registerCommand("check_shooter", CheckShooter(self.shooter, self.utilsys, self.timer))
        NamedCommands.registerCommand("check_arm", CheckArm(self.trapper, self.utilsys, self.timer))
        NamedCommands.registerCommand("check_intake", CheckIntake(self.intake, self.utilsys, self.timer))
        NamedCommands.registerCommand("master_caution", MasterCaution(self.leds, self.timer))
