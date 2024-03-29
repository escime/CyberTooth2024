from commands2 import Command, InterruptionBehavior
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.ledsubsystem import LEDs
from subsystems.intakesubsystem import IntakeSubsystem
from constants import TrapperConstants


class ClimbS1(Command):

    def __init__(self, trapper: TrapperSubsystem, leds: LEDs, intake: IntakeSubsystem):
        super().__init__()
        self.trapper = trapper
        self.leds = leds
        self.intake = intake
        self.addRequirements(intake)
        self.addRequirements(trapper)
        self.addRequirements(leds)

    def initialize(self):
        self.trapper.set_arm("stage")

    def execute(self):
        self.leds.rainbow_shift()
        self.intake.intake(0.5)
        if self.trapper.climb_encoder.getPosition() < TrapperConstants.climber_preset:
            self.trapper.run_climb(1)
        else:
            self.trapper.run_climb(0)

    def isFinished(self) -> bool:
        return False

    def getInterruptionBehavior(self) -> InterruptionBehavior:
        return InterruptionBehavior.kCancelSelf

    def end(self, interrupted: bool):
        self.trapper.run_climb(0)
        self.intake.intake(0)

