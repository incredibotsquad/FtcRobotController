package org.firstinspires.ftc.teamcode.VisionPathing;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Every tunable number for the VisionPathing package lives here, and nowhere else.
 *
 * Rules that keep this sane:
 *   - Nothing in this package writes to these fields at runtime EXCEPT calibration
 *     (TILT_DEG / FOCAL_PX), which does it on purpose so the result shows up in Panels.
 *     The old PollenIntake OpModes rewrote each other's tracker settings; this package never does.
 *   - Panels edits are live, but they reset on an app restart / redeploy. When a value is
 *     good, copy it back into this file.
 *   - Change one number at a time.
 *
 * Coordinate conventions used everywhere in this package:
 *   - ROBOT frame:  x = inches to the RIGHT of robot center, y = inches FORWARD of robot center.
 *   - FIELD frame:  Pedro Pathing's. Inches, origin bottom-left corner, heading in radians,
 *                   counter-clockwise positive, 0 = facing +x.
 *   - "Robot center" must be the same point your Pedro localizer reports.
 */
@Configurable
public class VisionConstants {

    // =====================================================================
    // GAME PIECES  (official: BIOBUZZ Competition Manual TU07, section 9.8)
    //   POLLEN = approx 2.8 in (7.1 cm) yellow polyethylene balls
    //   NECTAR = approx 3.6 in (9.1 cm) red / blue polyethylene balls
    //   "not perfectly spherical and may vary in size", hence the size tolerances below.
    // =====================================================================
    public static double POLLEN_DIAMETER = 2.8;          // in
    public static double NECTAR_DIAMETER = 3.6;          // in

    // =====================================================================
    // CAMERA HARDWARE
    // =====================================================================
    public static String WEBCAM_NAME = "Webcam 1";

    /** Capture size. FOCAL_PX scales with this: re-calibrate after ANY change. */
    public static final int IMG_W = 320;
    public static final int IMG_H = 240;

    /** Driver station / Panels preview frame rate. Read once at startup. */
    public static final int PANELS_STREAM_FPS = 10;
    public static boolean STREAM_TO_PANELS = true;

    // ---- Tape-measure numbers.  Every distance depends on these. ----
    public static double LENS_HEIGHT = 9.1;              // floor to lens center (in)          <- MEASURE
    public static double CAM_FORWARD_OFFSET = 0.0;       // lens forward of robot center (in), negative = behind  <- MEASURE (0 is suspicious)
    public static double CAM_RIGHT_OFFSET = 2.25;        // lens right of robot center (in)    <- MEASURE
    public static double CAM_YAW_DEG = 0.0;              // + = camera turned to the robot's right

    // ---- From calibration (press X or B during init, then copy the printed values here) ----
    public static double TILT_DEG = -25.59;              // NEGATIVE = tilted down
    public static double FOCAL_PX = 427.2;               // for 320x240 only

    // ---- Exposure.  Locked so the color ranges stay valid between venues. ----
    public static boolean MANUAL_EXPOSURE = true;
    public static long EXPOSURE_MS = 10;
    public static int GAIN = 100;

    /** Used only if the camera's frame timestamp is unusable: assumed capture-to-process delay. */
    public static double FRAME_LATENCY_S = 0.06;

    // =====================================================================
    // CALIBRATION  (one ball straight ahead at a measured distance)
    // =====================================================================
    public static boolean AUTO_CALIBRATE = false;        // true = calibrate every init. Off: it used to silently save garbage.
    public static double CAL_FLOOR_DIST = 30.0;          // floor point under the lens -> ball's floor contact (in)
    public static int CAL_SAMPLES = 15;                  // frames averaged
    public static double CAL_MAX_JITTER_PX = 2.0;        // reject if the ball wobbles more than this (std dev, px)
    public static double CAL_MAX_OFF_CENTER = 0.15;      // ball must be within this fraction of image width of center
    public static double CAL_FOCAL_MIN = 120, CAL_FOCAL_MAX = 900;   // sanity limits for a 320 px wide image
    public static double CAL_TILT_MIN = -70, CAL_TILT_MAX = 20;      // sanity limits (deg)

    // =====================================================================
    // COLOR  (YCrCb, 0-255.  128 = neutral on Cr and Cb.)
    //   Y  = brightness.   Cr = red(+) vs green(-).   Cb = blue(+) vs yellow(-).
    //   Use MASK_VIEW to see what each range lets through.
    // =====================================================================
    public static boolean DETECT_POLLEN = true;
    public static boolean DETECT_RED = true;
    public static boolean DETECT_BLUE = true;

    // ---- POLLEN (yellow): tuned values carried over from blobTrack ----
    public static int P_Y_MIN  = 32,  P_Y_MAX  = 255;
    public static int P_CR_MIN = 128, P_CR_MAX = 185;
    public static int P_CB_MIN = 0,   P_CB_MAX = 105;    // main yellow knob

    // ---- RED NECTAR: high Cr.  UNTUNED starting point, tune on real balls ----
    public static int R_Y_MIN  = 32,  R_Y_MAX  = 255;
    public static int R_CR_MIN = 150, R_CR_MAX = 255;    // raise if skin / wood / orange gets in
    public static int R_CB_MIN = 0,   R_CB_MAX = 135;
    /** Red and yellow overlap in Cr. True = any pixel that passes the Pollen range can never be red. */
    public static boolean RED_EXCLUDES_YELLOW = true;

    // ---- BLUE NECTAR: high Cb, low Cr (purple has high Cr, so CR_MAX rejects it).  UNTUNED ----
    public static int B_Y_MIN  = 32,  B_Y_MAX  = 255;
    public static int B_CR_MIN = 0,   B_CR_MAX = 125;
    public static int B_CB_MIN = 145, B_CB_MAX = 255;    // raise if purple gets in

    // =====================================================================
    // MASK CLEANUP
    // =====================================================================
    public static int BLUR = 5;                          // odd; 0 or 1 = off
    public static int OPEN_SIZE = 3;                     // deletes specks / thin strips
    public static int CLOSE_SIZE = 5;                    // fills holes inside balls
    /** Preview shows only one color's mask: 0 = off, 1 = Pollen, 2 = red, 3 = blue. */
    public static int MASK_VIEW = 0;

    // =====================================================================
    // DETECTION  (pixels)
    // =====================================================================
    public static double ROI_TOP_FRAC = 0.0;             // ignore image above this fraction (0 = whole frame)
    public static double MIN_RADIUS_PX = 4;
    public static double OVERLAP_FRAC = 0.5;             // higher = one ball less likely to get two circles
    public static double SLIVER_RATIO = 0.4;             // reject small leftovers touching a bigger ball
    public static int EDGE_MARGIN_PX = 2;                // reject balls cut off by ANY image edge (incl. top)
    public static int MAX_BALLS_PER_COLOR = 8;           // per color, so red can't use up blue's budget

    // ---- Real-world filters ----
    public static double SIZE_MIN = 0.6;                 // measured radius / expected radius
    public static double SIZE_MAX = 1.5;
    /** Looser for Pollen: missing an obstacle is worse than a false one (half-hidden balls look small). */
    public static double POLLEN_SIZE_MIN = 0.45;
    public static double MAX_DETECT_DIST = 96.0;         // ignore anything further from the robot (in); error grows with distance^2

    // =====================================================================
    // TRACKER  (field frame; balls are assumed to sit still unless pushed)
    // =====================================================================
    public static int CONFIRM_HITS = 3;                  // frames before a ball is trusted
    public static int MAX_VISIBLE_MISSES = 6;            // frames a ball may be missing WHILE IT SHOULD BE IN VIEW
    public static double MEMORY_S = 4.0;                 // forget a ball not seen for this long, even if out of view
    public static double TENTATIVE_LIFE_S = 0.5;         // unconfirmed balls die this fast if not re-seen
    public static double GATE_MIN = 3.0;                 // in; matching gate floor
    public static double GATE_SIGMAS = 3.0;              // gate = GATE_MIN + this * combined uncertainty
    public static double PIXEL_NOISE_PX = 1.5;           // assumed center noise; sets how much far readings are trusted
    public static double MEAS_SIGMA_MIN = 0.75;          // in; floor on measurement uncertainty (odometry, mount flex)
    public static double BALL_DRIFT_IPS = 2.0;           // in/s; how fast a ball's position is allowed to wander
    public static double FRESH_S = 0.25;                 // a ball seen within this many seconds counts as "visible now"

    // =====================================================================
    // ROBOT + INTAKE GEOMETRY  (all inches, robot frame)
    // =====================================================================
    public static double INTAKE_WIDTH = 16.0;            // intake mouth width                  <- given: ~16 in
    public static double ROBOT_WIDTH = 16.0;             // chassis width                       <- MEASURE
    public static double ROBOT_LENGTH = 16.0;            // chassis length                      <- MEASURE
    public static double FRONT_EXTENT = 8.0;             // robot center -> front of the intake <- MEASURE
    public static double INTAKE_CENTER_RIGHT = 0.0;      // intake center right of robot center
    /** Robot center -> ball center when the intake has the ball. Put a ball in the intake and measure. */
    public static double END_CENTER_TO_BALL = 6.0;

    // =====================================================================
    // PATH PLANNING  (NectarChaser)
    // =====================================================================
    public static double POLLEN_CLEARANCE = 1.5;         // extra gap between an obstacle and the edge of the robot/intake (in)
    public static boolean AVOID_OTHER_NECTAR = true;     // treat the other alliance's Nectar as obstacles too
    public static boolean AVOID_UNCONFIRMED = true;      // also dodge balls seen only once or twice (safer)
    public static double FINAL_APPROACH_LEN = 14.0;      // straight drive-in before the ball (in)
    public static double FINAL_LEG_SPEED = 0.45;         // fraction of max speed on the drive-in
    public static double APPROACH_STEP_DEG = 15.0;       // approach directions tried, in steps of this
    public static double APPROACH_MAX_DEG = 180.0;       // how far the approach may swing from straight-in
    public static double APPROACH_TURN_PENALTY = 0.15;   // in of extra path cost per degree off straight-in
    public static double DETOUR_MARGIN = 2.0;            // detour waypoints sit this far beyond the minimum clearance
    public static int MAX_DETOURS = 2;                   // waypoints allowed around obstacles per path
    public static double SAMPLE_SPACING = 1.0;           // in between collision-check samples

    // ---- Field walls.  Only turn on once the start pose below is real. ----
    public static boolean USE_FIELD_BOUNDS = false;
    public static double FIELD_SIZE = 144.0;
    public static double WALL_MARGIN = 9.0;              // keep robot center this far from the walls

    // =====================================================================
    // CHASE BEHAVIOR  (NectarChaser)
    // =====================================================================
    /** Which Nectar to collect: 0 = red, 1 = blue, 2 = either. */
    public static int TARGET_ALLIANCE = 0;
    public static double MAX_TARGET_DIST = 84.0;         // ignore Nectar further than this (in)
    public static int MAX_CANDIDATES = 4;                // nearest Nectar actually path-planned when choosing
    public static double SWITCH_MARGIN = 12.0;           // a new target must be this much cheaper (in) to steal the lock
    public static double FACE_TOL_DEG = 8.0;             // turn to face the target first, within this
    public static double FACE_TIMEOUT_S = 1.5;
    public static double SETTLE_S = 0.25;                // look after turning, before committing to a path
    public static double REPLAN_MIN_S = 0.35;            // never re-plan more often than this
    public static double REPLAN_TARGET_SHIFT = 2.0;      // re-plan if the target estimate moves this much (in)
    public static double NO_REPLAN_INSIDE = 4.0;         // stop re-planning for target shifts this close to the end (in)
    public static double CHASE_TIMEOUT_S = 10.0;         // safety timeout per target
    public static double COLLECT_DWELL_S = 0.4;          // sit at the end with the intake running
    public static double BLOCKED_RETRY_S = 0.5;          // when every path is blocked, look again this often
    public static double SEARCH_TURN_POWER = 0.0;        // rotate in place to look for Nectar; 0 = just wait
    /** Stop after this many. 0 = no limit.  G407: a robot may not CONTROL more than 4 scoring elements. */
    public static int MAX_TO_COLLECT = 0;

    // ---- Intake motor (optional).  "" = this package never touches the intake. ----
    public static String INTAKE_MOTOR = "intakeMotor";
    public static double INTAKE_POWER = 1.0;             // flip the sign if it spits instead of eats
    public static double INTAKE_ON_WITHIN = 18.0;        // run the intake when this close to the end point (in)

    // ---- Where the robot starts (Pedro field frame) ----
    public static double START_X = 72.0;
    public static double START_Y = 72.0;
    public static double START_HEADING_DEG = 90.0;
}
