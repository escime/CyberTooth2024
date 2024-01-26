import commands2
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard, DriverStation, Timer
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.ledsubsystem import LEDs
from commands.shoot_leds import ShootLEDs
from constants import VisionConstants
import math
from wpimath.controller import PIDController


class VisionSubsystem(commands2.SubsystemBase):
    limelight_table: NetworkTableInstance.getDefault().getTable("limelight")
    tv = 0.0
    tvf = 0.0
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
    timer = Timer()
    turn_to_target_controller = PIDController(VisionConstants.turnkP, 0, 0)
    approach_target_controller = PIDController(VisionConstants.rangekP, 0, 0)
    pipeline_id = 0
    vision_odo = False
    target_locked = False

    def __init__(self, robot_drive: DriveSubsystem) -> None:
        super().__init__()
        self.robot_drive = robot_drive  # This is structurally not great but necessary for certain features.
        self.limelight_table = NetworkTableInstance.getDefault().getTable("limelight")
        self.limelight_front = NetworkTableInstance.getDefault().getTable("llf")
        self.timer.start()
        self.record_time = self.timer.get()
        self.latency = 0

    def toggle_leds(self, on: bool):
        if on:
            self.limelight_table.putNumber("ledMode", 3.0)
            self.limelight_front.putNumber("ledMode", 3.0)
            return True
        else:
            self.limelight_table.putNumber("ledMode", 1.0)
            self.limelight_front.putNumber("ledMode", 1.0)
            return False

    def flash_leds(self, on: bool):
        if on:
            self.limelight_table.putNumber("ledMode", 2.0)
            self.limelight_front.putNumber("ledMode", 2.0)
        else:
            self.limelight_table.putNumber("ledMode", 1.0)
            self.limelight_front.putNumber("ledMode", 1.0)

    def update_values(self):
        """Update relevant values from LL NT to robot variables."""
        self.tv = self.limelight_table.getEntry("tv").getDouble(0)  # Get if AprilTag is visible.
        self.tvf = self.limelight_front.getEntry("tv").getDouble(0)  # Get if Note is visible.
        self.ta = self.limelight_front.getEntry("ta").getDouble(0)  # Get target area of Note.
        self.tl = self.limelight_table.getEntry("tl").getDouble(0.0)  # Get pipeline latency contribution.
        self.ty = self.limelight_table.getEntry("ty").getDouble(0.0)  # Get height of AprilTag relative to camera.
        self.tx = self.limelight_table.getEntry("tx").getDouble(0.0)  # Get angle offset from AprilTag.
        self.tag_id = self.limelight_table.getEntry("tid").getDouble(0)
        self.json_val = self.limelight_table.getEntry("json").getString("0")  # Grab the entire json pull as a string.
        first_index = str(self.json_val).find("\"ts\"")  # Locate the first string index for timestamp.
        adjusted_json = str(self.json_val)[first_index + 5:]  # Substring the JSON to remove everything before timestamp
        timestamp_str = adjusted_json[:adjusted_json.find(",")]  # Substring out the timestamp.
        try:
            self.timestamp = float(timestamp_str)  # Update timestamp if JSON parse is successful.
        except ValueError:
            self.timestamp = -1
        # Calculate latency based on Limelight Timestamp.
        self.latency = self.timer.getFPGATimestamp() - (self.tl/1000.0) - (self.timestamp/1000.0)

    def update_values_safe(self):
        """Update relevant values from LL NT to robot variables."""
        self.tv = self.limelight_table.getEntry("tv").getDouble(0)  # Get if AprilTag is visible.
        self.tvf = self.limelight_front.getEntry("tv").getDouble(0)  # Get if Note is visible.
        self.ta = self.limelight_front.getEntry("ta").getDouble(0)  # Get target area of Note.
        self.ty = self.limelight_table.getEntry("ty").getDouble(0.0)  # Get height of AprilTag relative to camera.
        self.tx = self.limelight_table.getEntry("tx").getDouble(0.0)  # Get angle offset from AprilTag.
        self.tag_id = self.limelight_table.getEntry("tid").getDouble(0)

    def has_targets(self) -> bool:
        """Checks if the limelight can see a target."""
        if self.tv == 1:
            return True
        else:
            return False

    def has_targets_f(self) -> bool:
        """Checks if the limelight can see a Note."""
        if self.tvf == 1:
            return True
        else:
            return False

    def vision_estimate_pose(self) -> Pose2d:
        """Returns limelight estimated robot pose."""
        botpose = self.limelight_table.getEntry("botpose_wpiblue").getDoubleArray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        bot_x = botpose[0]
        bot_y = botpose[1]
        rotation_z = (botpose[5] + 360) % 360

        return Pose2d(Translation2d(bot_x, bot_y), Rotation2d.fromDegrees(rotation_z))

#     def get_latency(self):
#         return Timer.getFPGATimestamp() - wpimath.units.millisecondsToSeconds(self.tl)

    def reset_hard_odo(self):
        """Reset robot odometry based on vision pose. Intended for use only during testing, since there is no auto
        to automatically update the initial pose and the software assumes (0, 0)."""
        # self.robot_drive.reset_odometry(self.vision_estimate_pose())
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.robot_drive.reset_odometry(Pose2d(Translation2d(8.12, 4), Rotation2d(0)))
        else:
            self.robot_drive.reset_odometry(Pose2d(Translation2d(8.12, 4), Rotation2d.fromDegrees(0)))

    def periodic(self) -> None:
        """Update vision variables and robot odometry as fast as scheduler allows."""
        if self.vision_odo:  # Enable vision-based odometry.
            if self.limelight_table.getNumber("camMode", -1) != 0:  # If camera not in vision mode,
                self.limelight_table.putNumber("camMode", 0)  # Put camera in vision mode.
            if self.limelight_table.getNumber("pipeline", 0) != 0:  # If camera not in pipeline 0,
                self.limelight_table.putNumber("pipeline", 0)  # Put camera in pipeline 0.
            if self.timer.get() - 0.5 > self.record_time:  # If it's been 0.5s since last update,
                self.update_values()  # Update limelight values.
                if self.has_targets():  # If an AprilTag is visible,
                    current_position = self.robot_drive.get_pose()  # Grab current pose.
                    vision_estimate = self.vision_estimate_pose()  # Estimate pose from vision.
                    if abs(current_position.x - vision_estimate.x) < 1 and \
                            abs(current_position.y - vision_estimate.y) < 1:  # Check if poses are within 1m.
                        self.robot_drive.add_vision(vision_estimate, self.latency)  # Add vision to kalman filter.
                self.record_time = self.timer.get()  # Reset timer.

        if not self.vision_odo:  # If robot is in targeting mode,
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:  # If on red alliance,
                if self.limelight_table.getNumber("pipeline", 0) != 2:  # If camera not in pipeline 2,
                    self.limelight_table.putNumber("pipeline", 2)  # Put camera in pipeline 2.
            else:  # Otherwise,
                if self.limelight_table.getNumber("pipeline", 0) != 1:  # If camera not in pipeline 1,
                    self.limelight_table.putNumber("pipeline", 1)  # Put camera in pipeline 1.
            if self.timer.get() - 0.5 > self.record_time:
                self.update_values_safe()  # Update all values.
                self.record_time = self.timer.get()

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

    def conditional_instant_update(self, drive: DriveSubsystem) -> None:
        self.update_values()
        if self.has_targets():
            drive.reset_odometry(self.vision_estimate_pose())

    def calcs_toggle(self):
        if self.calc_override:
            self.calc_override = False
        else:
            self.calc_override = True

    def pipeline_switch(self, pipeline_id: int):
        self.pipeline_id = pipeline_id

    def calculate_range_with_tag(self):
        """Range from target (for shooter)."""
        if self.tag_id in [1, 2, 3, 6, 7, 8]:
            angle_to_goal = (VisionConstants.rotation_from_horizontal + self.ty) * math.pi / 180
            target_range = (VisionConstants.tag_heights[self.tag_id] -
                            VisionConstants.lens_height) / math.atan(angle_to_goal)
        else:
            target_range = -1
        return target_range

    def rotate_to_target(self, drive: DriveSubsystem, x_speed: float, y_speed: float) -> None:
        """Aim at target (for shooter.)"""
        if self.has_targets():
            if self.tx < -VisionConstants.turn_to_target_error_max:
                rotate_output = self.turn_to_target_controller.calculate(0, self.tx) + VisionConstants.min_command
            elif self.tx > VisionConstants.turn_to_target_error_max:
                rotate_output = self.turn_to_target_controller.calculate(0, self.tx) - VisionConstants.min_command
            else:
                rotate_output = 0
            drive.drive(x_speed, y_speed, rotate_output, True)

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

    def range_and_turn_to_target(self, drive: DriveSubsystem, target_range: float) -> None:
        """Turn to target and approach a game piece."""
        if self.has_targets_f():
            rotate_output = self.turn_to_target_controller.calculate(0, self.tx)
            ranging = self.calculate_range_area()
            if ranging != -1:
                drive_output = self.approach_target_controller.calculate(target_range, ranging)
            else:
                drive_output = 0
            SmartDashboard.putNumber("Distance to NOTE", ranging)
            drive.drive(drive_output, 0, rotate_output, False)
        else:
            drive.drive(0, 0, 0, False)

    def range_to_angle(self):
        """Calculate shooter speed from range to target."""
        lookup_dist = [3.56, 0.80, 0.20]
        lookup_angle = [31, 64.5, 125]
        solution = -1
        for x in range(0, len(lookup_dist) - 1):
            if lookup_dist[x + 1] <= self.calculate_range_with_tag() < lookup_dist[x]:
                m = (lookup_angle[x + 1] - lookup_angle[x]) / (lookup_dist[x + 1] - lookup_dist[x])
                b = lookup_angle[x] - (m * lookup_dist[x])
                solution = (m * self.calculate_range_with_tag()) + b
        return solution

