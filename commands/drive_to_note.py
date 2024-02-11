import commands2
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.ledsubsystem import LEDs
from wpilib import Timer


class DriveToNote(commands2.Command):

    def __init__(self, drive: DriveSubsystem, intake: IntakeSubsystem,
                 vision: VisionSubsystem, trapper: TrapperSubsystem, leds: LEDs):
        super().__init__()
        self.drive = drive
        self.intake = intake
        self.vision = vision
        self.trapper = trapper
        self.leds = leds

        self.addRequirements(drive)
        self.addRequirements(intake)
        # self.addRequirements(vision)
        self.addRequirements(trapper)
        self.addRequirements(leds)

        self.timer = Timer()
        self.start_time = 0

    def initialize(self):
        self.timer.start()
        self.start_time = self.timer.get()

    def execute(self):
        self.intake.intake(1)
        self.trapper.advance_to_trapper()
        self.vision.range_and_turn_to_target(self.drive, 0)
        self.leds.flash_color([119, 247, 30], 2)

    def isFinished(self) -> bool:
        if self.trapper.get_note_acquired() or self.timer.get() - 3 > self.start_time:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.intake.intake(0)
        self.trapper.manual_trap(0)
        print("Drive to Note complete.")
