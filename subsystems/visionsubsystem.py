import commands2
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard, DriverStation, Timer
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from subsystems.drivesubsystem import DriveSubsystem
from constants import VisionConstants
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory
import math
from wpimath.controller import PIDController


class VisionSubsystem(commands2.SubsystemBase):
    limelight_table: NetworkTableInstance.getDefault().getTable("limelight")
    tv = 0.0
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

    def __init__(self, robot_drive: DriveSubsystem) -> None:
        super().__init__()
        self.robot_drive = robot_drive  # This is structurally not great but necessary for certain features.
        self.limelight_table = NetworkTableInstance.getDefault().getTable("limelight")
        # self.limelight_table = NetworkTableInstance.getDefault().getTable("limelight2")
        self.timer.start()
        self.record_time = self.timer.get()
        self.latency = 0

    def toggle_leds(self, on: bool):
        if on:
            self.limelight_table.putNumber("ledMode", 3.0)
            return True
        else:
            self.limelight_table.putNumber("ledMode", 1.0)
            return False

    def update_values(self):
        """Update relevant values from LL NT to robot variables."""
        self.tv = self.limelight_table.getEntry("tv").getDouble(0)  # Get "target acquired" boolean as a 1.0 or 0.0.
        self.ta = self.limelight_table.getEntry("ta").getDouble(0)  # Get "target area of image" as a double.
        self.tl = self.limelight_table.getEntry("tl").getDouble(0.0)  # Get pipeline latency contribution.
        self.ty = self.limelight_table.getEntry("ty").getDouble(0.0)
        self.tx = self.limelight_table.getEntry("tx").getDouble(0.0)
        self.tag_id = self.limelight_table.getEntry("tid").getDouble(0)
        self.json_val = self.limelight_table.getEntry("json").getString("0")  # Grab the entire json pull as a string.
        first_index = str(self.json_val).find("\"ts\"")  # Locate the first string index for timestamp.
        adjusted_json = str(self.json_val)[first_index + 5:]  # Substring the JSON to remove everything before timestamp
        timestamp_str = adjusted_json[:adjusted_json.find(",")]  # Substring out the timestamp.
        try:
            self.timestamp = float(timestamp_str)  # Update timestamp if JSON parse is successful.
        except ValueError:
            self.timestamp = -1
        self.latency = self.timer.getFPGATimestamp() - (self.tl/1000.0) - (self.timestamp/1000.0)

    def has_targets(self) -> bool:
        """Checks if the limelight can see a target."""
        if self.tv == 1:
            return True
        else:
            return False

    def vision_estimate_pose(self):
        """Acquires limelight estimated robot pose. Currently, pulls wpiblue botpose only."""
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
        self.robot_drive.reset_odometry(Pose2d(Translation2d(8.12, 4), Rotation2d(0)))

    def periodic(self) -> None:
        """Update vision variables and robot odometry as fast as scheduler allows."""
        # self.update_values()
        # Logic for manual/automatic camera swapping & vision tracking enable/disable
        SmartDashboard.putBoolean("Auto Camera Swap Enabled?", self.auto_cam_swap)
        if not DriverStation.isAutonomous() and not self.calc_override:  # Not in auto and not overriding calcs.
            if self.auto_cam_swap:  # Auto cam swap is on.
                if 90 < self.robot_drive.get_heading() % 360 <= 270:  # Robot is facing towards driver station.
                    self.pov = "front"
                else:  # Robot is facing away from driver station.
                    self.pov = "back"
            if self.pov == "front":  # Camera target POV is front.
                if self.limelight_table.getNumber("stream", -1) != 2:  # CHANGED THIS AND FOLLOWING LINE FROM 2 TO 0
                    self.limelight_table.putNumber("stream", 2)
            else:  # Camera target POV is back.
                if self.limelight_table.getNumber("stream", -1) != 1:  # CHANGED THIS AND FOLLOWING LINE FROM 1 TO 0
                    self.limelight_table.putNumber("stream", 1)
            if self.limelight_table.getNumber("camMode", -1) != 1:
                self.limelight_table.putNumber("camMode", 1)
        elif self.calc_override:  # Calc override is on.
            if self.limelight_table.getNumber("stream", -1) != 1:
                self.limelight_table.putNumber("stream", 1)
            if self.limelight_table.getNumber("camMode", -1) != 0:
                self.limelight_table.putNumber("camMode", 0)
            # If calc_override is TRUE, update odometry from camera every 0.5s.
            if self.timer.get() - 0.5 > self.record_time:
                self.update_values()
                if self.has_targets():
                    current_position = self.robot_drive.get_pose()
                    vision_estimate = self.vision_estimate_pose()
                    SmartDashboard.putString("Vision Estimated Pose", str(vision_estimate))

                    if abs(current_position.x - vision_estimate.x) < 1 and \
                            abs(current_position.y - vision_estimate.y) < 1:  # Sanity check for pose updates.
                        self.robot_drive.add_vision(vision_estimate, self.latency)
                self.record_time = self.timer.get()
        else:  # Robot is in auto.
            if self.limelight_table.getNumber("stream", -1) != 1:
                self.limelight_table.putNumber("stream", 1)
            if self.limelight_table.getNumber("camMode", -1) != 0:
                self.limelight_table.putNumber("camMode", 0)
        if self.limelight_table.getNumber("pipeline", 0) != self.pipeline_id:
            self.limelight_table.putNumber("pipeline", self.pipeline_id)

        # if self.has_targets():
            # current_position = self.robot_drive.get_pose()
            # vision_estimate = self.vision_estimate_pose()
            # SmartDashboard.putString("Vision Estimated Pose", str(vision_estimate))

            # if abs(current_position.x - vision_estimate.x) < 1 and \
            #         abs(current_position.y - vision_estimate.y) < 1:  # Sanity check for pose updates.
            #     self.robot_drive.add_vision(vision_estimate, self.timestamp)

    def update_target_tag(self, target: int) -> None:
        """Set the VisionSubsystem's target apriltag based on the red alliance equivalent tags."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            if target == 1:
                self.target_tag = 6
            elif target == 2:
                self.target_tag = 7
            elif target == 3:
                self.target_tag = 8
            elif target == 5:
                self.target_tag = 4
        else:
            self.target_tag = target

    def generate_path_to_tag(self) -> Trajectory:
        estimate_config = TrajectoryConfig(4, 3)
        end_pose = self.robot_drive.get_pose()
        if self.target_tag == 1:
            end_pose = Pose2d(14.67, 0.94, math.pi)
        elif self.target_tag == 2:
            end_pose = Pose2d(14.67, 2.62, math.pi)
        elif self.target_tag == 3:
            end_pose = Pose2d(14.67, 4.24, math.pi)
        elif self.target_tag == 4:
            end_pose = Pose2d(15.64, 6.68, math.pi)
        elif self.target_tag == 5:
            end_pose = Pose2d(0.69, 6.68, 180)
        elif self.target_tag == 6:
            end_pose = Pose2d(1.68, 4.24, 180)
        elif self.target_tag == 7:
            end_pose = Pose2d(1.68, 2.62, 180)
        elif self.target_tag == 8:
            end_pose = Pose2d(1.68, 0.94, 180)
        path = TrajectoryGenerator.generateTrajectory([self.robot_drive.get_pose(), end_pose], estimate_config)
        return path

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

    def flash_leds(self) -> None:
        self.limelight_table.putNumber("ledMode", 2)

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
        if self.tag_id in [1, 2, 3, 6, 7, 8]:
            angle_to_goal = (VisionConstants.rotation_from_horizontal + self.ty) * math.pi / 180
            target_range = (VisionConstants.tag_heights[self.tag_id] -
                            VisionConstants.lens_height) / math.atan(angle_to_goal)
        else:
            target_range = -1
        return target_range

    def rotate_to_target(self, drive: DriveSubsystem, x_speed: float, y_speed: float) -> None:
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
        if self.has_targets():
            rotate_output = self.turn_to_target_controller.calculate(0, self.tx)
            ranging = self.calculate_range_area()
            if ranging != -1:
                drive_output = self.approach_target_controller.calculate(target_range, ranging)
            else:
                drive_output = 0
            SmartDashboard.putNumber("Calculated Range", ranging)
            drive.drive(drive_output, 0, rotate_output, False)
        else:
            drive.drive(0, 0, 0, False)
