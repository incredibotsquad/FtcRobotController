# Pollen Path Planning Utilities

Two static helper methods added to `blobTrack.java` for making safe autonomous driving decisions.

---

## `isPathBlocked`

```java
blobTrack.PathBlockResult result = blobTrack.isPathBlocked(
    target.x, target.y,     // robot-relative target position (in)
    nectarPositions,         // List<double[]> of {x, y} obstacle floor positions (in)
    9.0                      // corridor half-width — robot half-width + margin (in)
);
if (result.blocked) {
    // steer away before chasing
    double turn = result.suggestedTurnDegrees; // positive = right, negative = left
}
```

### What it does

Tests whether any obstacle (nectar balls) falls inside the straight-line **corridor** from the robot to the target pollen group.

```
         [robot]
            |  <-- corridor half-width on each side
    X       |          X = obstacle OUTSIDE corridor (ignored)
       N    |     N = obstacle INSIDE corridor (BLOCKED)
            |
         [target]
```

Each obstacle is checked in two steps:

1. **Along-axis projection** — compute how far down the path the obstacle sits.
   Obstacles behind the robot (`along < 0`) or past the target (`along > pathLen`) are ignored.

2. **Perpendicular distance** — compute the signed lateral offset from the path centre line.
   Positive = obstacle is to the **left** of the path.
   If `|perpendicular| < corridorHalfWidth`, the obstacle is blocking.

### Suggested turn direction

When blocked, the method returns the minimum heading change (degrees) that shifts the corridor enough to clear the nearest blocker:

```
suggestedTurnDegrees > 0  →  turn RIGHT  (obstacle was on the left)
suggestedTurnDegrees < 0  →  turn LEFT   (obstacle was on the right)
suggestedTurnDegrees = 0  →  not blocked
```

The magnitude is computed as:
```
clearOffset = |perp| + corridorHalfWidth + 50% margin
turn        = atan2(clearOffset, halfPathDistance)  ×  direction
```

A larger `clearOffset` (obstacle deep inside corridor) produces a bigger turn. Obstacles far away need less turn than close ones.

### Getting nectar positions

`blobTrack.PollenProcessor` detects **pollen** (yellow). Nectar (red/blue) must come from a
separate source. Options:

- **BioBuzzProcessor** (if available): call `getLatest()`, filter by `Kind.RED_NECTAR` /
  `Kind.BLUE_NECTAR`, project each result through `blobTrack.Geometry.project(px, py, radius)`
  to get robot-relative floor `{x, y}`.
- A second `VisionProcessor` running on the same `VisionPortal` with nectar-specific YCrCb ranges.

### Tuning `corridorHalfWidth`

| Value | Meaning |
|-------|---------|
| 7–9 in | Robot half-width (≈ 18" robot / 2 = 9) — only avoids direct collisions |
| 12–15 in | Adds a safety margin for sensor noise and ball roll |

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
        nectarFloorPositions,  // built from BioBuzzProcessor each frame
        9.0
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
