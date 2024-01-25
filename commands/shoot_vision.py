import commands2
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.ledsubsystem import LEDs
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from wpilib import Timer


class ShootVision(commands2.Command):
    def __init__(self, shooter: ShooterSubsystem, vision: VisionSubsystem,
                 drive: DriveSubsystem, leds: LEDs, intake: IntakeSubsystem, trapper: TrapperSubsystem):
        super().__init__()
        self.shooter = shooter
        self.vision = vision
        self.drive = drive
        self.leds = leds
        self.intake = intake
        self.trapper = trapper
        self.addRequirements(shooter)
        self.addRequirements(vision)
        self.addRequirements(drive)
        self.addRequirements(leds)
        self.addRequirements(intake)
        self.addRequirements(trapper)
        self.timer = Timer()
        self.start_time = 0

    def initialize(self):
        self.timer.start()
        self.vision.target_locked = False
        self.vision.vision_odo = False

    def execute(self) -> None:
        self.vision.aim_and_fire(self.drive, self.shooter, self.leds, self.intake, self.trapper)
        if self.vision.target_locked:
            self.start_time = self.timer.get()

    def isFinished(self) -> bool:
        # TODO replace with a different condition later.
        if self.timer.get() - 1 > self.start_time:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.shooter.set_known_setpoint("readied")
        self.intake.intake(0)
        self.trapper.manual_trap(0)
        self.vision.vision_odo = True
