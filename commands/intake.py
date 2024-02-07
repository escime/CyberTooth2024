import commands2
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from constants import IntakeConstants


class Intake(commands2.Command):

    def __init__(self, intake: IntakeSubsystem, trapper: TrapperSubsystem):
        super().__init__()
        self.intake = intake
        self.trapper = trapper
        self.addRequirements(intake)
        self.addRequirements(trapper)

    def initialize(self):
        self.trapper.stow()

    def execute(self):
        self.intake.intake(IntakeConstants.intake_speed)
        self.trapper.advance_to_trapper()

    def isFinished(self) -> bool:
        return self.trapper.get_note_acquired()

    def end(self, interrupted: bool):
        self.intake.intake(0)
        self.trapper.manual_trap(0)
