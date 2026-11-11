# Pollen Path Planning Utilities

Two static helper methods added to `blobTrack.java` for making safe autonomous driving decisions.

---

## `isPathBlocked`

```java
blobTrack.PathBlockResult result = blobTrack.isPathBlocked(
    target.x, target.y,  // robot-relative target position (in)
    nectarPositions       // List<double[]> of {x, y} floor positions from Geometry.project()
);
if (result.blocked) {
    double turn = result.suggestedTurnDegrees; // positive = turn right, negative = turn left
}
```

### Setup: measure and set the robot width constants

Before using `isPathBlocked`, measure the robot and set these two constants in `blobTrack.java`:

```java
public static double ROBOT_HALF_WIDTH_LEFT  = 9.0;  // robot centre to left  physical edge (in)
public static double ROBOT_HALF_WIDTH_RIGHT = 9.0;  // robot centre to right physical edge (in)
```

These are independent of the camera position. `Geometry.project()` already adds `CAM_RIGHT_OFFSET`
back to every detected obstacle position, so all obstacle `{x, y}` values are already in
**robot-centre coordinates**. The function simply checks whether those coordinates fall inside
the robot's physical footprint.

### What it does

Tests whether any obstacle (nectar balls) falls inside the straight-line **corridor** from the
robot to the target pollen group.

```
         [robot centre]
   ←left─┤       ├─right→    physical edges (ROBOT_HALF_WIDTH_LEFT / RIGHT)
         |       |
    X    |       |   X        X = outside footprint, ignored
      N  |       |            N = inside footprint, BLOCKED
         |       |
      [target]
```

Each obstacle is checked in two steps:

1. **Along-axis projection** — how far down the path is the obstacle?
   Obstacles behind the robot (`along < 0`) or past the target (`along > pathLen`) are ignored.

2. **Perpendicular offset** — signed lateral distance from the robot's centre-line path.
   Positive = obstacle is to the **left**. The check is asymmetric:

   | Obstacle side | Blocking condition |
   |---|---|
   | Left  (`perp ≥ 0`) | `perp < ROBOT_HALF_WIDTH_LEFT` |
   | Right (`perp < 0`) | `\|perp\| < ROBOT_HALF_WIDTH_RIGHT` |

### Suggested turn direction

```
suggestedTurnDegrees > 0  →  turn RIGHT  (blocker was left of centre)
suggestedTurnDegrees < 0  →  turn LEFT   (blocker was right of centre)
suggestedTurnDegrees = 0  →  path is clear
```

Magnitude = `atan2(clearanceNeeded, distanceToBlocker)`.
The deeper the obstacle is inside the footprint, the larger the required turn.
Obstacles further away need a smaller heading change than close ones.

### Getting nectar positions

`blobTrack.PollenProcessor` detects **pollen** (yellow). Nectar (red/blue) must come from a
separate source. Options:

- **BioBuzzProcessor** (if available): call `getLatest()`, filter by `Kind.RED_NECTAR` /
  `Kind.BLUE_NECTAR`, project each result through `blobTrack.Geometry.project(px, py, radius)`
  to get robot-relative floor `{x, y}`.
- A second `VisionProcessor` on the same `VisionPortal` with nectar-specific YCrCb ranges.

---

## `requiresCrossField`

```java
boolean illegal = blobTrack.requiresCrossField(
    target.x, target.y,         // robot-relative target position (in)
    odo.getX(), odo.getY(),     // robot field position from OdometrySubsystem (in)
    odo.getHeadingDegrees(),    // robot heading CCW from field +X (deg)
    72.0,                       // boundary at field centre (half of 144")
    isRedAlliance               // true if robot started below Y = 72"
);
if (illegal) {
    // skip this group — it's on the other alliance's side
}
```

### What it does

Converts the robot-relative target position to **absolute field coordinates**, then checks whether
that position is on the opposing alliance's half of the field.

### Coordinate transform

```
field +X →
field +Y ↑

Robot at (robotFieldX, robotFieldY) heading θ CCW from field +X.
  Robot forward  (+y robot) = field direction  (cos θ,  sin θ)
  Robot right    (+x robot) = field direction  (sin θ, −cos θ)

fieldTargetX = robotFieldX + targetY·cos θ + targetX·sin θ
fieldTargetY = robotFieldY + targetY·sin θ − targetX·cos θ
```

### Boundary check

```
robotBelowBoundary = true  →  robot's half is fieldY < 72"
    returns true if fieldTargetY > 72"   (target is across the line)

robotBelowBoundary = false →  robot's half is fieldY > 72"
    returns true if fieldTargetY < 72"
```

### Important: heading convention

`robotHeadingDeg` must be in the **same convention as your OdometrySubsystem** — CCW from
field +X, where 0° means the robot faces the field +X direction.  If your odometry uses a
different zero reference (e.g. 0° = pointing toward the opposing wall), adjust the heading
before passing it in:

```java
double adjustedHeading = odo.getHeadingDegrees() + YOUR_OFFSET_DEG;
blobTrack.requiresCrossField(tx, ty, fx, fy, adjustedHeading, 72.0, true);
```

---

## Example: combined usage in `moveToBall`

```java
// In HUNTING or CHASE state, before committing to a target group:
Group candidate = tracker.getGroups().isEmpty() ? null : tracker.getGroups().get(0);
if (candidate != null) {

    // 1. Check for illegal field crossing
    boolean crossesField = blobTrack.requiresCrossField(
        candidate.x, candidate.y,
        odo.getX(), odo.getY(), odo.getHeadingDegrees(),
        72.0, isRedAlliance
    );

    // 2. Check if nectar is blocking the path
    blobTrack.PathBlockResult path = blobTrack.isPathBlocked(
        candidate.x, candidate.y,
        nectarFloorPositions   // built from BioBuzzProcessor each frame
    );

    if (crossesField) {
        // Skip — this pollen is on the other side
    } else if (path.blocked) {
        // Adjust heading first, then re-evaluate next frame
        turn = path.suggestedTurnDegrees * KP_TURN_ADJUST;
    } else {
        // Safe to chase
        state = State.CHASE;
    }
}
```
