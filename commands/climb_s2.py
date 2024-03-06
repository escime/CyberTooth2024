from commands2 import Command, InterruptionBehavior
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.ledsubsystem import LEDs
from subsystems.drivesubsystem import DriveSubsystem
from constants import TrapperConstants, DriveConstants


class ClimbS2(Command):

    def __init__(self, trapper: TrapperSubsystem, leds: LEDs, drive: DriveSubsystem):
        super().__init__()
        self.trapper = trapper
        self.leds = leds
        self.drive = drive
        self.addRequirements(drive)
        self.addRequirements(trapper)
        self.addRequirements(leds)

    def initialize(self):
        self.trapper.set_arm("trap")

    def execute(self):
        self.leds.rainbow_shift()
        if self.trapper.climb_encoder.getPosition() < TrapperConstants.climber_preset_2:
            self.trapper.run_climb(1)
        else:
            self.trapper.run_climb(0)
        self.drive.drive(0.05 * DriveConstants.kMaxSpeed, 0, 0, False)

    def isFinished(self) -> bool:
        return False

    def getInterruptionBehavior(self) -> InterruptionBehavior:
        return InterruptionBehavior.kCancelSelf

    def end(self, interrupted: bool):
        self.trapper.run_climb(0)

