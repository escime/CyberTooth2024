import commands2
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.ledsubsystem import LEDs
from commands.shoot import Shoot
from commands.shoot_leds import ShootLEDs
from constants import VisionConstants
from wpilib import Timer


class ShootVision(commands2.Command):
    def __init__(self, bypass_timer: bool, shooter: ShooterSubsystem, vision: VisionSubsystem,
                 drive: DriveSubsystem, intake: IntakeSubsystem, trapper: TrapperSubsystem, leds: LEDs):
        super().__init__()
        self.bypass_timer = bypass_timer
        self.shooter = shooter
        self.vision = vision
        self.drive = drive
        self.intake = intake
        self.trapper = trapper
        self.leds = leds
        self.addRequirements(shooter)
        # self.addRequirements(vision)
        self.addRequirements(drive)
        self.addRequirements(intake)
        self.addRequirements(trapper)
        self.timer = Timer()
        self.start_time = 1000
        self.overrun_time = self.timer.get()
        self.target_locked = False

    def initialize(self):
        self.timer.start()
        self.vision.vision_odo = False
        self.overrun_time = self.timer.get()
        self.target_locked = False

    def execute(self) -> None:
        if self.vision.has_targets() and self.vision.range_to_angle() != -1:
            self.shooter.set_unknown_setpoint(self.vision.no_sight_range_to_angle(),
                                              VisionConstants.shooter_default_speed)
            self.vision.rotate_to_target_all_locations(self.drive, 0, 0)
            if self.shooter.get_ready_to_shoot() and -VisionConstants.turn_to_target_error_max < self.vision.tx < \
                    VisionConstants.turn_to_target_error_max:
                self.target_locked = True

    def isFinished(self) -> bool:
        if not self.bypass_timer:
            if self.target_locked or self.timer.get() - 3 > self.overrun_time:
                return True
            else:
                return False
        else:
            if self.target_locked:
                return True
            else:
                return False

    def end(self, interrupted: bool):
        # ShootLEDs(self.leds, "fast")
        # Shoot("readied", False, self.shooter, self.intake, self.trapper)
        self.drive.drive_2ok(0, 0, 0, False)
        print("ShootVision complete.")
