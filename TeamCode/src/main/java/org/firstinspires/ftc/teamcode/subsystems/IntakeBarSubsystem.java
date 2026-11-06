package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The intake bar: drops down in front of the robot to help pull POLLEN out of a FLOWER's bottom
 * Retrieval Opening (manual 9.7), and raises back up out of the way. Driven by two positional
 * servos, one on each side of the bar, that always move together.
 *
 * The two servos are mounted mirrored, so the same bar angle is a different servo position on each
 * side. Each servo therefore has its own RAISED and LOWERED positions, and an in-between bar
 * position is the same fraction of the way between them on both servos. That keeps the servos from
 * fighting each other through the bar.
 *
 * Tuning (ServoPositionTuner): tune ONE servo at a time. An uncommanded servo is unpowered, so the
 * other side follows freely. Find each servo's RAISED and LOWERED positions with the bar at the
 * real end points, and copy them in below. If the two sets don't agree, the servos stall against
 * each other: if a servo buzzes or gets hot with the bar at rest, recheck its positions.
 *
 * Rules:
 *  - G304.H / G403: nothing is commanded in the constructor, so the bar can't move during INIT.
 *    The TeleOp calls raise() during INIT so the bar is held up. That is only legal because the bar
 *    already rests there (holding a position isn't motion), so RAISED must match where the bar
 *    physically sits. The bar must start RAISED, inside the 18 in starting configuration (R102).
 *  - R105 / G416: the expanded size limit must be enforced mechanically, not by these positions.
 *  - G418: only POLLEN may be removed from a FLOWER's bottom. Don't use the bar to pull NECTAR out.
 */
@Configurable
public class IntakeBarSubsystem extends SubsystemBase {

    /** Hardware names; must match the Robot Controller configuration. */
    public static String LEFT_SERVO_NAME = "intakeBarLeftServo";
    public static String RIGHT_SERVO_NAME = "intakeBarRightServo";

    // PLACEHOLDERS: tune with ServoPositionTuner (see class comment).
    public static double LEFT_LOWERED = 0.650;
    public static double LEFT_RAISED = 0.310;
    public static double RIGHT_LOWERED = 0.600;
    public static double RIGHT_RAISED = 0.940;

    /**
     * Time for the bar to travel all the way from RAISED to LOWERED. Servos give no feedback, so
     * isAtTarget() is an estimate: a move of part of the travel waits that part of this time.
     */
    public static long FULL_TRAVEL_MS = 400;

    public enum State {
        /** Not commanded since the OpMode started; servos unpowered. */
        UNKNOWN,
        RAISED,
        LOWERED,
        /** Somewhere between, via setBarFraction(). */
        PARTIAL
    }

    private final SimpleServo leftServo;
    private final SimpleServo rightServo;
    private final TelemetryManager telemetry;

    private State state = State.UNKNOWN;
    /** 0 = raised, 1 = lowered; NaN = never commanded. */
    private double fraction = Double.NaN;
    private long moveDoneAtMs = 0;

    public IntakeBarSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        leftServo = new SimpleServo(hardwareMap, LEFT_SERVO_NAME, 0, 270);
        rightServo = new SimpleServo(hardwareMap, RIGHT_SERVO_NAME, 0, 270);
        // Deliberately no setPosition here: see G304.H in the class comment.
    }

    public void raise() {
        setBarFraction(0.0);
    }

    public void lower() {
        setBarFraction(1.0);
    }

    /** Lowers the bar unless it is already (going) down, in which case raises it. */
    public void toggle() {
        if (state == State.LOWERED) raise();
        else lower();
    }

    /**
     * Moves both servos to the same fraction of their travel: 0 = RAISED, 1 = LOWERED.
     * Clamped to [0, 1].
     */
    public void setBarFraction(double f) {
        f = Math.max(0.0, Math.min(1.0, f));

        // Estimate travel time from how far the bar has to move (full travel if never commanded).
        double distance = Double.isNaN(fraction) ? 1.0 : Math.abs(f - fraction);
        moveDoneAtMs = nowMs() + Math.round(distance * FULL_TRAVEL_MS);

        fraction = f;
        state = f == 0.0 ? State.RAISED : f == 1.0 ? State.LOWERED : State.PARTIAL;

        leftServo.setPosition(LEFT_LOWERED + f * (LEFT_RAISED - LEFT_LOWERED));
        rightServo.setPosition(RIGHT_LOWERED + f * (RIGHT_RAISED - RIGHT_LOWERED));
    }

    public State getState() {
        return state;
    }

    /** Commanded bar fraction (0 = raised, 1 = lowered), or NaN if never commanded. */
    public double getBarFraction() {
        return fraction;
    }

    public boolean isLowered() {
        return state == State.LOWERED;
    }

    public boolean isRaised() {
        return state == State.RAISED;
    }

    /** Estimated: true once the bar has had time to reach its last command. False if never commanded. */
    public boolean isAtTarget() {
        return state != State.UNKNOWN && nowMs() >= moveDoneAtMs;
    }

    @Override
    public void periodic() {
        telemetry.addData("Intake bar", state
                + (state == State.UNKNOWN ? "" : String.format(" (%.2f)%s", fraction, isAtTarget() ? "" : " moving")));
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
