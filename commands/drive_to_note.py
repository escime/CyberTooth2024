from commands2 import Command
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from constants import DriveConstants
from wpilib import Timer


class DriveToNote(Command):

    def __init__(self, drive: DriveSubsystem, intake: IntakeSubsystem,
                 vision: VisionSubsystem, trapper: TrapperSubsystem, timer: Timer):
        super().__init__()
        self.drive = drive
        self.intake = intake
        self.vision = vision
        self.trapper = trapper

        self.addRequirements(drive)
        self.addRequirements(intake)
        # self.addRequirements(vision)
        self.addRequirements(trapper)

        self.timer = timer
        self.start_time = 0

    def initialize(self):
        self.start_time = self.timer.get()

    def execute(self):
        self.intake.intake(1)
        self.trapper.advance_to_trapper()
        self.vision.forward_and_turn_to_target(self.drive, DriveConstants.kMaxSpeed * 0.1)

    def isFinished(self) -> bool:
        if self.trapper.get_note_acquired() or self.timer.get() - 3 > self.start_time:
            return True
        else:
            return False

    def end(self, interrupted: bool):
        self.intake.intake(0)
        self.trapper.manual_trap(0)
        print("Drive to Note complete.")
