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
    def __init__(self, shooter: ShooterSubsystem, vision: VisionSubsystem,
                 drive: DriveSubsystem, intake: IntakeSubsystem, trapper: TrapperSubsystem, leds: LEDs):
        super().__init__()
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

    def initialize(self):
        self.timer.start()
        self.vision.target_locked = False
        self.vision.vision_odo = False
        self.overrun_time = self.timer.get()
        self.shooter.spin_up(VisionConstants.shooter_default_speed)

    def execute(self) -> None:
        if self.vision.has_targets() and self.vision.range_to_angle() != -1:
            self.shooter.set_angle(self.vision.range_to_angle())
            self.vision.rotate_to_target(self.drive, 0, 0)
            if self.shooter.get_ready_to_shoot() and -VisionConstants.turn_to_target_error_max < self.vision.tx < \
                    VisionConstants.turn_to_target_error_max:
                self.vision.target_locked = True

    def isFinished(self) -> bool:
        if self.vision.target_locked or self.timer.get() - 3 > self.overrun_time:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        # ShootLEDs(self.leds, "fast")
        # Shoot("readied", False, self.shooter, self.intake, self.trapper)
        print("ShootVision complete.")
