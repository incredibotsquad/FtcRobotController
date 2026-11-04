# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

FTC (FIRST Tech Challenge) robot controller app for the BIOBUZZ 2026-2027 season, built on the official FTC SDK 12.0. It's an Android app. All team code lives in the `TeamCode` module. `FtcRobotController` is the SDK module and should be left alone except for pulling in upstream SDK updates. Its `external/samples` folder holds reference OpModes you can copy into TeamCode.

## Build

- `./gradlew :TeamCode:assembleDebug` builds the APK. Use this to check that changes compile.
- `./gradlew installDebug` installs to a connected Robot Controller (Control Hub over adb/Wi-Fi).
- There are no unit tests and no lint setup. You can only verify behavior on the physical robot.
- Don't edit `build.common.gradle` or the top-level `build.gradle`; the SDK owns them. Add team dependencies to `TeamCode/build.gradle` or `build.dependencies.gradle`. The latter already has the extra Maven repos (bylazar, dairy.foundation).

## Libraries

- **FTCLib** (`org.ftclib.ftclib:core`): command-based framework. Subsystems extend `SubsystemBase`, and `periodic()` runs each loop through the `CommandScheduler`. Uses FTCLib hardware wrappers (`MotorEx`, `SimpleServo`) and geometry (`Pose2d`, `Rotation2d`, `Translation2d`).
- **Pedro Pathing 3.0** (`com.pedropathing:revhub`, `com.pedropathing:tuning`): path following. Uses its own `com.pedropathing.math.Pose`, which is a different type from FTCLib's `Pose2d`.
- **Panels** (`com.bylazar:fullpanels`): dashboard. `@Configurable` exposes a class's `public static` (non-final) fields for live tuning. Subsystems take a Panels `TelemetryManager` (not the SDK `Telemetry`).

## Architecture

- `subsystems/`: one FTCLib subsystem per mechanism (Drive, Intake, LaunchGate, Launch, Odometry). Each constructor takes `(HardwareMap, TelemetryManager)`, except `OdometrySubsystem`. Hardware device names are string literals in the constructors and must match the Robot Controller configuration on the robot.
  - `DriveSubsystem` runs everything through Pedro's `Follower` (from `pedro/Constants.create()`), which is the only owner of the drive motors and the Pinpoint. Its `periodic()` calls `follower.update()`. `DriveRobotCommand` drives with `follower.manual()` (forward, strafe left, turn CCW). While `LaunchBallsCommand` runs it holds the robot's pose (`follower.hold`) against defense. Moving a stick breaks out of the hold, and `DriveSubsystem.LAUNCH_HOLD_ENABLED` turns the feature off.
  - `OdometrySubsystem` reads the pose from `DriveSubsystem` rather than the Pinpoint. It is constructed with `(DriveSubsystem, TelemetryManager)` and must be registered after `DriveSubsystem` (FTCLib runs `periodic()` in registration order). Its `periodic()` copies the pose, estimates velocity and acceleration, and writes the pose to `CrossOpModeStorage`. Pose resets go through `follower.setPose()`. Pinpoint pod offsets live in `pedro/Constants.localizerConfig`. `updatePoseFromLimelight` applies vision corrections from `RelocalizeWithLimelightCommand`.
  - `LimelightSubsystem` (Limelight 3A, fixed to the chassis) uses each HIVE tag's robot-space position, not MegaTag, because the tags move when a HIVE TIPS. Tag height tells whether a CELL is up or down and updates `HiveState`. Tag geometry is in `common/HiveTags`. `DriveSubsystem.isEffectivelyStationary()` gates when that relocalization runs.
  - `LaunchSubsystem` runs a dual flywheel (PID + `SimpleMotorFeedforward`, RPM converted to ticks/sec using GoBILDA BARE CPR) and a turret servo (5-turn, 112:29 gear ratio). There is no adjustable visor/hood. `isReadyToLaunch()` checks flywheel RPM and turret position plus `shotSolutionReady`, and `periodic()` sets the alignment indicator light from the result.
  - Indicator "lights" are servo-PWM driven. Servo positions map to colors through the constants in each subsystem.
- `common/CrossOpModeStorage`: static fields that keep pose, alliance color, and turret position across OpModes (e.g. auto → teleop). Launch target coordinates are in `common/LaunchTargets` (4 HIVE CELL positions; the `USE_VISION_SELECTOR` flag picks between the nearest-CELL selector with hysteresis and `HiveStateSelector`, which aims at the up CELL tracked by `common/HiveState`). Field coordinates are in inches on a 144×144 field.
- `common/BIOBUZZ_Manual_notes.md`: full digest of the BIOBUZZ competition manual (TU07), organized by section and tagged [CODE]/[STRATEGY]/[DERIVED]/[AMBIGUOUS]. Check game rules, field geometry and AprilTag positions here. [DERIVED] values came from the drawings and need verifying.
- `pedro/`: Pedro Pathing setup. `Constants.create()` is meant to build the `Follower` (drivetrain + localizer + Foresight) but is currently a stub that returns `null`. `pedro/procedures/` holds Pedro's autotune `Procedure` classes (localizer tuners for Pinpoint/OTOS/OctoQuad/two-/three-wheel, mecanum, Foresight, plus `Tests`).
- No TeleOp or Autonomous OpModes exist yet. New ones go under TeamCode with `@TeleOp`/`@Autonomous`; see `TeamCode/.../teamcode/readme.md` for SDK naming conventions.
