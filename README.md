# CyberTooth 2024
Complete software for CyberTooth's 2024 FIRST Robotics Competition robot, Metronome.

Software features:
- Integration with PathPlanner for fast and simple autonomous development.
- Custom swerve drive implementation for L3 Mk4i SDS modules with NEO 1.1s and CanCoders.
  - Auto Targeting of SPEAKER AprilTags using auto-switching pipelines.
  - Odometry updates from periodic vision measurements.
  - Closed loop turning and location snap control.
  - On-the-fly pathplanning for STAGE alignment.
- Simplified implementation of SysID tests for fast robot bringup.
- Closed loop shooter control with lookup table to enable accurate scoring at multiple ranges. Lookup table uses near-point interpolation to avoid inaccuracies created by system non-linearity.
- Automated TRAP and ONSTAGE scoring.
- Subystem collision avoidance safeties in both automatic and manual modes.
- Single-button scoring in both the SPEAKER and AMP.

Coming soon:
- AMP alignment path planning.
- Shooting while moving(?)
- Improved autonomous tuning