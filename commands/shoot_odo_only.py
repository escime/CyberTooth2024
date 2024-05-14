from commands2 import Command
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.trappersubsystem import TrapperSubsystem
from subsystems.ledsubsystem2 import LEDs
from constants import VisionConstants
from wpilib import Timer


class ShootVisionOdo(Command):
    def __init__(self, bypass_timer: bool, shooter: ShooterSubsystem, vision: VisionSubsystem,
                 drive: DriveSubsystem, intake: IntakeSubsystem, trapper: TrapperSubsystem, leds: LEDs, timer: Timer):
        super().__init__()
        self.bypass_timer = bypass_timer
        self.shooter = shooter
        self.vision = vision
        self.drive = drive
        self.intake = intake
        self.trapper = trapper
        self.leds = leds
        self.addRequirements(leds)
        self.addRequirements(shooter)
        # self.addRequirements(vision)
        self.addRequirements(drive)
        self.addRequirements(intake)
        self.addRequirements(trapper)
        self.timer = timer
        self.start_time = 10000
        self.overrun_time = self.timer.get()
        self.target_locked = False
        self.ready_buffer = [False] * 8

    def initialize(self):
        self.vision.vision_odo_manual(True)
        self.overrun_time = self.timer.get()
        self.target_locked = False
        self.leds.set_state("align")

    def execute(self) -> None:
        if not self.vision.vision_shot_bypass:
            if self.vision.range_to_angle_m(self.drive) != -1:
                self.shooter.set_unknown_setpoint(self.vision.range_to_angle_m(self.drive),
                                                  VisionConstants.shooter_default_speed)
            self.vision.align_to_speaker_odo(0, 0, self.drive)
            if self.shooter.get_ready_to_shoot() and self.vision.get_aligned_odo(2, self.drive):
                self.ready_buffer[0] = True
            else:
                self.ready_buffer[0] = False
            self.ready_buffer = self.ready_buffer[1:] + self.ready_buffer[:1]
            if all(self.ready_buffer):
                print("TARGET LOCKED!")
                self.target_locked = True
            self.leds.set_misalignment(self.vision.alpha, self.drive.get_heading_odo().degrees())
        else:
            self.shooter.set_known_setpoint("podium")
            self.drive.drive(0, 0, 0, False)
            self.leds.set_misalignment(0, 0)

    def isFinished(self) -> bool:
        if not self.vision.vision_shot_bypass:
            if not self.bypass_timer:
                if self.target_locked or self.timer.get() - 1 > self.overrun_time:
                    return True
                else:
                    return False
            else:
                if self.target_locked:
                    return True
                else:
                    return False
        else:
            if self.shooter.get_ready_to_shoot():
                return True
            else:
                return False

    def end(self, interrupted: bool):
        if interrupted:
            self.shooter.set_known_setpoint("readied")
        self.vision.vision_odo_manual(True)
        self.target_locked = False
        self.leds.set_state("default")
