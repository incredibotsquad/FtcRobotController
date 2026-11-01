# HiveSubsystem — BIOBuzz HIVE Vision

## What this does

`HiveSubsystem` drives the Limelight 3A for all HIVE-related vision in BIOBuzz. Every scheduler tick it reads all visible AprilTag clusters, determines which HIVE cells are scorable or tipped, and exposes that as a clean API for commands and OpModes.

**Important:** BIOBuzz has **no fixed wall tags** — there are no field-perimeter AprilTags for robot localization. All 16 tags (IDs 30–45) are mounted on the four HIVE cells and move when a HIVE tips. The old `LimelightSubsystem` field-localization approach does not apply to this game.

---

## The BIOBuzz HIVE — physical layout

There are two HIVEs on the field, one per alliance. Each HIVE is a bi-stable structure on a pivot with **two CELLS**. Only one cell per HIVE can face upward at a time. When enough pollen/nectar is launched into the upward cell, the HIVE tips — the cell that was up goes down and vice versa. Each tip is worth 20 points.

Each CELL has a **4-tag AprilTag cluster** on its underside. The four clusters and their tag IDs:

| HiveCell constant | Alliance | Tag IDs | Physical position |
|---|---|---|---|
| `RED_FAR`   | Red  | 30–33 | Red cell, far side (opposite audience) |
| `RED_NEAR`  | Red  | 34–37 | Red cell, audience side |
| `BLUE_NEAR` | Blue | 38–41 | Blue cell, audience side |
| `BLUE_FAR`  | Blue | 42–45 | Blue cell, far side (opposite audience) |

---

## How tipped state is detected

The Limelight returns a `Pose3D` for every visible tag, including its orientation relative to the robot. The **roll** of that pose tells you whether the cell is facing up or down.

```
|roll| < 90°   →  cell is facing UP   →  UPRIGHT_SCORABLE
|roll| >= 90°  →  cell is facing DOWN →  TIPPED_NOT_SCORABLE
```

This threshold comes directly from the FTC game manual. `HiveSubsystem` averages the roll across all visible tags in a cluster each frame, so a 4-tag cluster gives a much more stable reading than a single tag would.

---

## Why 4 tags per cell

1. **Angle coverage** — a single tag becomes undetectable when viewed edge-on. With 4 tags spread across the cell, at least one is readable from any position on the field.
2. **Roll accuracy** — roll is the hardest axis to measure from a single tag at distance. Four physically separated tags give the solver much stronger parallax.
3. **Cluster pose** — the Limelight solves all visible tags as a single rigid body for a more accurate 3D estimate.
4. **Redundancy** — if one tag is covered by a pollen ball or scuffed in competition, three remain.
5. **Cell identity** — each cluster has a unique ID range, so even one visible tag uniquely identifies which of the four physical cells you're looking at.

---

## Limelight Manager setup

1. Open **Limelight Manager** in a browser (default `http://limelight.local:5801`).
2. Create a new pipeline. Set:
   - **Pipeline type:** `Fiducial Tracking (AprilTag)`
   - **Tag family:** `36h11` — this is the family BIOBuzz uses
   - **Multi-tag pose estimation:** **Enabled** — this lets the Limelight solve all 4 cluster tags together for higher accuracy
3. Save the pipeline. Note the pipeline **index** (0, 1, 2…). Pass this to `HiveSubsystem` at construction.
4. You do **not** need separate pipelines per alliance. One AprilTag pipeline sees all tags 30–45.
5. If you also run a Python pollen-detection pipeline (for `BioBuzzLimelight`), put it on a **different index** and call `hiveSubsystem.setPipeline(index)` to switch between them.
6. Tune **exposure and white balance** for the HIVE lighting conditions. The HIVE is elevated and may be lit differently from the floor.

---

## Robot code integration

### Instantiation (already done in `Incredibot.java`)

```java
// Pipeline index 0 = the AprilTag HIVE pipeline you configured in Limelight Manager
hiveSubsystem = new HiveSubsystem(hwMap, telemetry, 0);
hiveSubsystem.start(CrossOpModeStorage.allianceColor);
```

`start()` switches to the HIVE pipeline and calls `limelight.start()`. Call it once after the alliance color is known (i.e., in the `Incredibot` constructor, which reads from `CrossOpModeStorage`).

### Registration

`HiveSubsystem` is already added to the `register()` call in `Incredibot.initCommon()`. This ensures `periodic()` is called every scheduler tick, which is where all detection logic runs.

---

## API reference

### Setup

```java
// Switch alliance and pipeline, restart streaming
hiveSubsystem.start(AllianceColors.RED);
hiveSubsystem.start(AllianceColors.BLUE, 2); // explicit pipeline index

// Switch to a different pipeline (e.g. pollen detection)
hiveSubsystem.setPipeline(1);
// Switch back
hiveSubsystem.setPipeline(0);
```

### State queries (call every loop or inside commands)

```java
// Is any cell for this alliance currently visible?
boolean visible = hiveSubsystem.hasCellVisible(AllianceColors.RED);

// State of whichever cell is currently in view
HiveSubsystem.CellState state = hiveSubsystem.getActiveCellState(AllianceColors.RED);
// Returns: UPRIGHT_SCORABLE, TIPPED_NOT_SCORABLE, or UNKNOWN

// State of a specific cell by enum constant
HiveSubsystem.CellState redFar = hiveSubsystem.getCellState(HiveSubsystem.HiveCell.RED_FAR);

// Which cell is currently the active scoring target?
HiveSubsystem.HiveCell active = hiveSubsystem.getActiveScoringCell(AllianceColors.RED);
// Returns null if no scorable cell visible

// How many tags from this cluster are visible (0–4)
int tagCount = hiveSubsystem.getVisibleTagCount(HiveSubsystem.HiveCell.RED_NEAR);
```

### Tip event detection

```java
// True for exactly ONE loop tick when any cell for this alliance
// transitions between UPRIGHT_SCORABLE and TIPPED_NOT_SCORABLE
if (hiveSubsystem.justTipped(AllianceColors.RED)) {
    // Re-aim the turret to the new upward cell immediately
}
```

### Targeting

```java
// Horizontal bearing to the active scorable cell, relative to robot forward
// Negative = left, positive = right. Null when no target visible.
Double bearing = hiveSubsystem.getTargetBearingDegrees();

// Horizontal distance to the active scorable cell in inches
Double distance = hiveSubsystem.getTargetDistanceInches();

// Full 3D pose of the best visible tag in robot-relative coordinates
// Use for precise turret angle + flywheel velocity beyond bearing/distance alone
Pose3D tagPose = hiveSubsystem.getActiveCellTagPose();
```

### Telemetry

```java
// Add all cell states and targeting data to the DS dashboard
hiveSubsystem.addTelemetry();
```

---

## Usage patterns

### Autonomous — wait for a scorable cell before driving in

```java
// In an auto command's initialize() or execute():
if (hiveSubsystem.hasCellVisible(alliance)
        && hiveSubsystem.getActiveCellState(alliance) == CellState.UPRIGHT_SCORABLE) {
    Double distance = hiveSubsystem.getTargetDistanceInches();
    // Use distance to decide approach path
}
```

### Autonomous — re-aim after a tip

```java
// In execute() of a continuous auto command:
if (hiveSubsystem.justTipped(alliance)) {
    // The HIVE just flipped — the new upward cell is ready
    // Recalculate turret/drive path to the new active cell
    Double newBearing = hiveSubsystem.getTargetBearingDegrees();
}
```

### Teleop — driver feedback on hive state

```java
// In an OpMode loop or periodic command:
hiveSubsystem.addTelemetry();
// Driver Station shows each cell's UPRIGHT_SCORABLE / TIPPED_NOT_SCORABLE / UNKNOWN
// and the current bearing/distance to the active target
```

### Targeting integration with the launcher

```java
// Inside a command that aims the turret:
Double bearing  = hiveSubsystem.getTargetBearingDegrees();
Double distance = hiveSubsystem.getTargetDistanceInches();

if (bearing != null && distance != null) {
    // bearing  → turret servo angle (instead of odometry-only calculation)
    // distance → flywheel RPM lookup table
}
```

The bearing from the Limelight is a **direct measurement** of where the cell is relative to the robot, not a dead-reckoned odometry estimate. This makes it more accurate than the pure-odometry approach currently in `LaunchReadinessCommand`, especially after the robot has drifted or been jostled.

---

## Migration from LimelightSubsystem

`LimelightSubsystem` has been replaced by `HiveSubsystem` throughout the codebase. The key differences:

| Old (`LimelightSubsystem`) | New (`HiveSubsystem`) |
|---|---|
| Used tag IDs 20 (blue) and 24 (red) — wrong game | Uses tag IDs 30–45 — correct for BIOBuzz |
| `setAlliance(boolean isRed)` | `start(AllianceColors)` |
| `getLatestFieldPose()` — relied on fixed wall tags | `getLatestFieldPose()` returns null (no wall tags in BIOBuzz) |
| No tipped-state detection | Full cluster roll → CellState per cell |
| No bearing/distance API | `getTargetBearingDegrees()`, `getTargetDistanceInches()` |

`RelocalizeCommand` and `LaunchReadinessCommand` both still compile and run. Since `ENABLE_RELOCALIZATION` and `ENABLE_TURRET_VISION_CORRECTION` are already `false`, neither command does anything different — they just use `HiveSubsystem` as the dependency type instead of the old class.
