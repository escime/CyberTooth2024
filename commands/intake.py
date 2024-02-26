from commands2 import Command
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from constants import IntakeConstants
from wpilib import Timer


class Intake(Command):

    def __init__(self, intake: IntakeSubsystem, trapper: TrapperSubsystem, bypass: bool, timer: Timer):
        super().__init__()
        self.intake = intake
        self.trapper = trapper
        self.bypass = bypass
        self.addRequirements(intake)
        self.addRequirements(trapper)
        self.timer = timer
        self.start_time = 0

    def initialize(self):
        self.start_time = self.timer.get()
        self.trapper.stow()

    def execute(self):
        self.intake.intake(IntakeConstants.intake_speed)
        self.trapper.advance_to_trapper()

    def isFinished(self) -> bool:
        if not self.bypass:
            if self.trapper.get_note_acquired() or (self.timer.get() - 5 > self.start_time):
                return True
            else:
                return False
        else:
            if self.trapper.get_note_acquired():
                return True
            else:
                return False

    def end(self, interrupted: bool):
        self.intake.intake(0)
        self.trapper.manual_trap(0)
        # if interrupted:
        #     print("Intake interrupted!")
        # else:
        #     print("Intake complete!")
