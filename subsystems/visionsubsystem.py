import commands2
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard, DriverStation, Timer
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from constants import VisionConstants
import math
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile


class VisionSubsystem(commands2.Subsystem):
    lookup_distance_m = [4.5, 4.12, 3.70, 2.70, 2.40, 1.89, 1.70, 1.25, 0]
    lookup_distance_est = [60, 57, 55, 50, 45, 40, 30, 20]
    lookup_angle = [0.766, 0.763, 0.758, 0.745, 0.740, 0.730, 0.720, 0.705, 0.705]

    # lookup_dist = [65.02, 60.10, 58, 55.07, 50.0, 49]
    # lookup_angle = [0.78, 0.77, 0.76, 0.758, 0.749, 0.747]
    # lookup_dist_m = [5, 4.83, 3.78, 3.53, 3.0866, 2.4222]
    # lookup_angle = [0.79, 0.78, 0.77, 0.76, 0.758, 0.749]

    tv = 0.0
    # tvf = 0.0
    ta = 0.0
    tl = 0.0
    ty = 0.0
    tx = 0.0
    tag_id = 1
    json_val = {}
    timestamp = 0
    target_led_mode = 1
    target_tag = 2
    pov = "back"
    auto_cam_swap = True
    calc_override = False
    turn_to_target_controller = PIDController(VisionConstants.turnkP, VisionConstants.turnkI, VisionConstants.turnkD)
    approach_target_controller = PIDController(VisionConstants.rangekP, 0, 0)
    shoot_while_move_controller = PIDController(0.2, 0, 0)
    mp_ttt_controller = ProfiledPIDController(VisionConstants.mp_ttt_kp, VisionConstants.mp_ttt_ki,
                                              VisionConstants.mp_ttt_kd, TrapezoidProfile.Constraints(2.5, 2.5))
    pipeline_id = 0
    vision_odo = True
    target_locked = False
    alpha = 0
    vision_shot_bypass = False
    # txf = 0.0

    def __init__(self, timer: Timer) -> None:
        super().__init__()
        self.timer = timer
        self.limelight_table = NetworkTableInstance.getDefault().getTable("limelight")
        # self.limelight_front = NetworkTableInstance.getDefault().getTable("limelight-front")
        self.record_time = self.timer.get()
        self.latency = 0

    def toggle_leds(self, on: bool):
        if on:
            self.limelight_table.putNumber("ledMode", 3.0)
            # self.limelight_front.putNumber("ledMode", 3.0)
            return True
        else:
            self.limelight_table.putNumber("ledMode", 1.0)
            # self.limelight_front.putNumber("ledMode", 1.0)
            return False

    def flash_leds(self, on: bool):
        if on:
            self.limelight_table.putNumber("ledMode", 2.0)
            # self.limelight_front.putNumber("ledMode", 2.0)
        else:
            self.limelight_table.putNumber("ledMode", 1.0)
            # self.limelight_front.putNumber("ledMode", 1.0)

    def update_values(self):
        """Update relevant values from LL NT to robot variables."""
        self.tv = self.limelight_table.getEntry("tv").getDouble(0)  # Get if AprilTag is visible.
        # self.tvf = self.limelight_front.getEntry("tv").getDouble(0)  # Get if Note is visible.
        # self.txf = self.limelight_front.getEntry("tx").getDouble(0)
        # self.ta = self.limelight_front.getEntry("ta").getDouble(0)  # Get target area of Note.
        self.tl = self.limelight_table.getEntry("tl").getDouble(0.0)  # Get pipeline latency contribution.
        self.ty = self.limelight_table.getEntry("ty").getDouble(0.0)  # Get height of AprilTag relative to camera.
        self.tx = self.limelight_table.getEntry("tx").getDouble(0.0)  # Get angle offset from AprilTag.
        self.tag_id = self.limelight_table.getEntry("tid").getDouble(0)

    def update_values_safe(self):
        """Update relevant values from LL NT to robot variables."""
        self.tv = self.limelight_table.getEntry("tv").getDouble(0)  # Get if AprilTag is visible.
        # self.tvf = self.limelight_front.getEntry("tv").getDouble(0)  # Get if Note is visible.
        # self.txf = self.limelight_front.getEntry("tx").getDouble(0)
        # self.ta = self.limelight_front.getEntry("ta").getDouble(0)  # Get target area of Note.
        self.ty = self.limelight_table.getEntry("ty").getDouble(0.0)  # Get height of AprilTag relative to camera.
        self.tx = self.limelight_table.getEntry("tx").getDouble(0.0)  # Get angle offset from AprilTag.
        self.tag_id = self.limelight_table.getEntry("tid").getDouble(0)  # Get which tag is currently in view.

    def has_targets(self) -> bool:
        """Checks if the limelight can see a target."""
        if self.tv == 1:
            return True
        else:
            return False

    # def has_targets_f(self) -> bool:
    #     """Checks if the limelight can see a Note."""
    #     if self.tvf == 1:
    #         return True
    #     else:
    #         return False

    def vision_estimate_pose(self) -> Pose2d:
        """Returns limelight estimated robot pose."""
        botpose = self.limelight_table.getEntry("botpose_wpiblue").getDoubleArray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        bot_x = botpose[0]
        bot_y = botpose[1]
        rotation_z = (botpose[5] + 360) % 360
        self.timestamp = self.timer.getFPGATimestamp() - (botpose[6] / 1000.0)

        return Pose2d(Translation2d(bot_x, bot_y), Rotation2d.fromDegrees(rotation_z))

#     def get_latency(self):
#         return Timer.getFPGATimestamp() - wpimath.units.millisecondsToSeconds(self.tl)

    def periodic(self) -> None:
        """Update vision variables and robot odometry as fast as scheduler allows."""
        if self.vision_odo:  # Enable vision-based odometry.
            if self.limelight_table.getNumber("camMode", -1) != 0:  # If camera not in vision mode,
                self.limelight_table.putNumber("camMode", 0)  # Put camera in vision mode.
            if self.limelight_table.getNumber("pipeline", 0) != 0:  # If camera not in pipeline 0,
                self.limelight_table.putNumber("pipeline", 0)  # Put camera in pipeline 0.
            if self.timer.get() - 0.2 > self.record_time:  # If it's been 0.2s since last update,
                self.update_values()  # Update limelight values.
                self.record_time = self.timer.get()  # Reset timer.

        if not self.vision_odo:  # If robot is in targeting mode,
            if self.timer.get() - 0.1 > self.record_time:
                if DriverStation.getAlliance() == DriverStation.Alliance.kRed:  # If on red alliance,
                    if self.limelight_table.getNumber("pipeline", 0) != 2:  # If camera not in pipeline 2,
                        self.limelight_table.putNumber("pipeline", 2)  # Put camera in pipeline 2.
                else:  # Otherwise,
                    if self.limelight_table.getNumber("pipeline", 0) != 1:  # If camera not in pipeline 1,
                        self.limelight_table.putNumber("pipeline", 1)  # Put camera in pipeline 1.
                self.update_values_safe()  # Update all values.
                self.record_time = self.timer.get()

        SmartDashboard.putBoolean("Targets Detected?", self.has_targets())
        SmartDashboard.putNumber("Range from Apriltag", self.calculate_range_with_tag())
        SmartDashboard.putNumber("Target Shooter Angle", self.range_to_angle())
        SmartDashboard.putNumber("Alpha", self.alpha)
        SmartDashboard.putBoolean("Vision Targeting Overridden?", self.vision_shot_bypass)
        # SmartDashboard.putNumber("Range from Note", self.calculate_range_area())

    def toggle_camera(self) -> None:
        if self.pov == "front":
            self.pov = "back"
        else:
            self.pov = "front"

    def toggle_auto_cam_swap(self) -> None:
        if self.auto_cam_swap:
            self.auto_cam_swap = False
        else:
            self.auto_cam_swap = True

    def instant_update(self) -> Pose2d:
        self.update_values()
        return self.vision_estimate_pose()

    def calcs_toggle(self):
        if self.calc_override:
            self.calc_override = False
        else:
            self.calc_override = True

    def pipeline_switch(self, pipeline_id: int):
        self.pipeline_id = pipeline_id

    def vision_odo_toggle(self):
        if self.vision_odo:
            self.vision_odo = False
        else:
            self.vision_odo = True

    def vision_odo_manual(self, on: bool):
        self.vision_odo = on

    def calculate_range_with_tag(self):
        """Range from target (for shooter)."""
        if self.has_targets():
            if int(self.tag_id) in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]:
                angle_to_goal = (VisionConstants.rotation_from_horizontal + self.ty) * math.pi / 180
                target_range = (VisionConstants.tag_heights[int(self.tag_id) - 1] -
                                VisionConstants.lens_height) / math.atan(angle_to_goal)
            else:
                target_range = -1
            return target_range
        else:
            return -1

    def calculate_range_area(self):
        """This is intended for 'bad' ranging using area for something like closing to a game piece."""
        lookup_area_percent = [3.56, 0.80, 0.20]
        lookup_distance_in = [31, 64.5, 125]
        solution = -1
        for x in range(0, len(lookup_area_percent) - 1):
            if lookup_area_percent[x + 1] <= self.ta < lookup_area_percent[x]:
                m = (lookup_distance_in[x + 1] - lookup_distance_in[x]) / (lookup_area_percent[x + 1] -
                                                                           lookup_area_percent[x])
                b = lookup_distance_in[x] - (m * lookup_area_percent[x])
                solution = (m * self.ta) + b
        return solution

    # def range_and_turn_to_target(self, drive: DriveSubsystem, target_range: float) -> None:
    #     """Turn to target and approach a game piece."""
    #     if self.has_targets_f():
    #         rotate_output = self.turn_to_target_controller.calculate(0, self.tx)
    #         ranging = self.calculate_range_area()
    #         if ranging != -1:
    #             drive_output = self.approach_target_controller.calculate(target_range, ranging)
    #         else:
    #             drive_output = 0
    #         SmartDashboard.putNumber("Distance to NOTE", ranging)
    #         drive.drive_2ok(drive_output, 0, rotate_output, False)
    #     else:
    #         drive.drive_2ok(0, 0, 0, False)

    # def forward_and_turn_to_target(self, drive: DriveSubsystem, speed: float) -> None:
    #     """Turn towards a target and drive forward at a constant, set speed. Designed for GP pickup."""
    #     if self.has_targets_f():
    #         rotate_output = self.turn_to_target_controller.calculate(0, self.txf)
    #         drive.drive_2ok(speed, 0, rotate_output, False)
    #     else:
    #         drive.drive_2ok(speed * 0.75, 0, 0, False)

    def range_to_angle(self):
        """Calculate shooter speed from range to target."""
        lookup_dist = self.lookup_distance_est
        lookup_angle = self.lookup_angle
        if self.has_targets():
            if lookup_dist[-1] <= self.calculate_range_with_tag() <= lookup_dist[0]:
                solution = -1
                for x in range(0, len(lookup_dist) - 1):
                    if lookup_dist[x + 1] <= self.calculate_range_with_tag() < lookup_dist[x]:
                        m = (lookup_angle[x + 1] - lookup_angle[x]) / (lookup_dist[x + 1] - lookup_dist[x])
                        b = lookup_angle[x] - (m * lookup_dist[x])
                        solution = (m * self.calculate_range_with_tag()) + b
                return solution
            else:
                return -1
        else:
            return -1

    def toggle_vision_shot_bypass(self):
        if self.vision_shot_bypass:
            self.vision_shot_bypass = False
        else:
            self.vision_shot_bypass = True

    def range_to_angle_rand(self, dist: float) -> float:
        # lookup_dist = [4.465628, 3.469694, 3.2434, 2.720572, 2.0826]
        # lookup_angle = [0.78, 0.77, 0.76, 0.758, 0.75]
        lookup_dist = self.lookup_distance_m
        lookup_angle = self.lookup_angle
        if lookup_dist[-1] <= dist <= lookup_dist[0]:
            solution = -1
            for x in range(0, len(lookup_dist) - 1):
                if lookup_dist[x + 1] <= dist < lookup_dist[x]:
                    m = (lookup_angle[x + 1] - lookup_angle[x]) / (lookup_dist[x + 1] - lookup_dist[x])
                    b = lookup_angle[x] - (m * lookup_dist[x])
                    solution = (m * dist) + b
            return solution
        else:
            return -1
