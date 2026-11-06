package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;

@Configurable
public class LaunchSubsystem extends SubsystemBase {
//
    private final MotorEx leftLaunchMotor;
    private final MotorEx rightLaunchMotor;

    private final SimpleServo turretServo;
    private final SimpleServo alignmentIndicatorLight;

    /** Flywheel counts as "at speed" within this many RPM of the target. */
    public static double TARGET_RPM_TOLERANCE = 50;
    private static final double TURRET_POSITION_TOLERANCE = 0.05;

    // ------------------------------------------------------------------ flywheel control
    // Gains act on ticks/sec error (GoBILDA BARE, 28 CPR). Power is in [0, 1].
    //
    // Two sources of gains:
    //  - FLYWHEEL_* below: the single set used by updateFlywheel(rpm) (no distance), i.e. what
    //    FlywheelTuner edits live.
    //  - The GAIN_TABLE_* arrays: one row per launch distance, used by updateFlywheel(rpm, distance)
    //    from LaunchReadinessCommand. Tune each row with FlywheelTuner at that distance's RPM, then
    //    copy the FLYWHEEL_* values it shows into that row.
    public static double FLYWHEEL_P = 0.08;
    public static double FLYWHEEL_I = 0.0;
    public static double FLYWHEEL_D = 0.0;
    /** Static friction: power needed to just start turning. */
    public static double FLYWHEEL_KS = 0.16;
    /** Power per tick/sec needed to hold a speed, measured at NOMINAL_VOLTAGE. */
    public static double FLYWHEEL_KV = 0.0005;

    /**
     * Gain table, indexed by distance (inches, robot center to CELL). All arrays are rows of the
     * same table: same length, GAIN_TABLE_DISTANCES_IN sorted ascending. Between rows each gain is
     * linearly interpolated; outside the table the nearest end row is used. Suggest keeping the
     * distances the same as LaunchReadinessCommand.SHOT_DISTANCES_IN so each RPM has its own row.
     * PLACEHOLDERS: every row starts as a copy of the single FLYWHEEL_* set.
     */
    public static boolean USE_GAIN_TABLE = true;
    //match these values to LaunchReadinessCommand.SHOT_DISTANCES_IN
    public static double[] GAIN_TABLE_DISTANCES_IN = {46, 47, 52, 66, 88};
    public static double[] GAIN_TABLE_P  = {0.025, 0.025, 0.025, 0.03, 0.035};
    public static double[] GAIN_TABLE_I  = {0.0, 0.0, 0.0, 0.0, 0.0};
    public static double[] GAIN_TABLE_D  = {0.0, 0.0, 0.0, 0.0, 0.0};
    public static double[] GAIN_TABLE_KS = {0.16, 0.16, 0.16, 0.16, 0.16};
    public static double[] GAIN_TABLE_KV = {0.00032419, 0.00032419, 0.00032419, 0.00032419, 0.00032419};

    /** Run full power while more than this far below target (fast spin-up and shot recovery). */
    public static boolean FLYWHEEL_BANG_BANG_ENABLED = true;
    public static double FLYWHEEL_BANG_BANG_THRESHOLD_RPM = 150;

    /** Scale feedforward by NOMINAL_VOLTAGE / battery voltage so kV holds as the battery sags. */
    public static boolean FLYWHEEL_VOLTAGE_COMPENSATION = true;
    /** Battery voltage kS/kV were tuned at. */
    public static double NOMINAL_VOLTAGE = 12.0;
    /** Voltage reads are slow hub calls (not bulk cached), so sample at this interval. */
    public static long VOLTAGE_SAMPLE_MS = 250;

    private volatile double targetRPM;
    private volatile double targetTurretPos;
    private volatile boolean shotSolutionReady = true;
    private volatile boolean shotPossible = true;

    public static double ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT = 0.3;    //RED
    public static double ROBOT_ALIGNED_TO_SHOOT_LIGHT = 0.5;    //GREEN
    public static double ROBOT_ALIGNMENT_NOT_POSSIBLE_LIGHT = 0;

    public static double TURRET_MIN = 0.2;
    public static double TURRET_MID = 0.5;
    public static double TURRET_MAX = 1.0;

    /*
     * MECHANICAL CALCULATION:
     * Ratio: 82 teeth (Turret) / 24 teeth (Servo) = 3.417
     * Servo: GoBILDA 5-Turn = 1800 degrees of total travel (0.0 to 1.0)
     *
     * Formula:
     * (TargetDegrees * Ratio) / TotalServoRange
     */
    public static double GEAR_RATIO = 82.0 / 24.0;
    // Servo's rated travel. Usable travel is limited by TURRET_MIN..TURRET_MAX; the hub's default
    // pulse range shortens it further (see LaunchReadinessCommand.TURRET_SCALING_FACTOR).
    public static double TOTAL_SERVO_RANGE = 1800;

    private final PIDController flywheelPID = new PIDController(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D);
    private SimpleMotorFeedforward flywheelFF = new SimpleMotorFeedforward(FLYWHEEL_KS, FLYWHEEL_KV);
    private double ffKs = FLYWHEEL_KS;
    private double ffKv = FLYWHEEL_KV;
    private double activeP = FLYWHEEL_P, activeI = FLYWHEEL_I, activeD = FLYWHEEL_D;

    private final VoltageSensor batteryVoltageSensor;
    private double batteryVoltage = NOMINAL_VOLTAGE;
    private long lastVoltageSampleMs = 0;

    private double lastFlywheelPower = 0;
    private boolean inBangBang = false;

    /** Turret lock: see setLaunchReadinessLock(). */
    private boolean isLaunchReadinessLocked = false;

    public static boolean SKIP_FLYWHEEL = false;

    private TelemetryManager telemetry;

    public LaunchSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        leftLaunchMotor = new MotorEx(hardwareMap, "leftLaunchMotor", Motor.GoBILDA.BARE);
        leftLaunchMotor.setRunMode(Motor.RunMode.RawPower);
        leftLaunchMotor.setInverted(true);

        rightLaunchMotor = new MotorEx(hardwareMap, "rightLaunchMotor", Motor.GoBILDA.BARE);
        rightLaunchMotor.setRunMode(Motor.RunMode.RawPower);

        turretServo = new SimpleServo(hardwareMap, "turretServo", 0, 1800);
        alignmentIndicatorLight = new SimpleServo(hardwareMap, "alignmentIndicatorLight", 0, 270);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;
    }

    /**
     * Turret lock. When locked, LaunchReadinessCommand moves the turret to TURRET_MID and keeps it
     * there instead of tracking. Everything else keeps running live: target selection, flywheel RPM
     * for the current distance, and shot validity. Readiness ignores turret aim while locked
     * (fully manual turret: aim by turning the robot).
     */
    public void setLaunchReadinessLock(boolean locked) {
        this.isLaunchReadinessLocked = locked;
    }

    public boolean isLaunchReadinessLocked() {
        return isLaunchReadinessLocked;
    }

    /**
     * Runs one step of flywheel control. Call EVERY loop; the flywheel only reacts when this runs.
     *  - Far below target (more than FLYWHEEL_BANG_BANG_THRESHOLD_RPM): full power. This is what
     *    makes spin-up and recovery after a shot fast.
     *  - Near target: feedforward (voltage compensated) + PID holds the speed.
     *  - Power is limited to [0, 1]: the flywheel coasts down instead of being braked in reverse.
     * Uses the single FLYWHEEL_* gain set (FlywheelTuner).
     */
    public void updateFlywheel(double targetRPM) {
        applyGains(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_KS, FLYWHEEL_KV);
        runFlywheel(targetRPM);
    }

    /**
     * Same as updateFlywheel(rpm), but gains come from the GAIN_TABLE_* row(s) for this distance to
     * the goal. Falls back to the FLYWHEEL_* set when USE_GAIN_TABLE is off or the table is empty.
     */
    public void updateFlywheel(double targetRPM, double distanceIn) {
        int n = gainTableRows();
        if (!USE_GAIN_TABLE || n == 0) {
            updateFlywheel(targetRPM);
            return;
        }
        applyGains(
                interpolate(GAIN_TABLE_P, distanceIn, n),
                interpolate(GAIN_TABLE_I, distanceIn, n),
                interpolate(GAIN_TABLE_D, distanceIn, n),
                interpolate(GAIN_TABLE_KS, distanceIn, n),
                interpolate(GAIN_TABLE_KV, distanceIn, n));
        runFlywheel(targetRPM);
    }

    /** Usable rows: the shortest of the GAIN_TABLE_* arrays (a mismatch shows up in telemetry). */
    private static int gainTableRows() {
        return Math.min(GAIN_TABLE_DISTANCES_IN.length,
                Math.min(Math.min(GAIN_TABLE_P.length, GAIN_TABLE_I.length),
                        Math.min(GAIN_TABLE_D.length, Math.min(GAIN_TABLE_KS.length, GAIN_TABLE_KV.length))));
    }

    /** Linear interpolation of one gain column over distance, clamped to the end rows. */
    private static double interpolate(double[] values, double distanceIn, int n) {
        double[] d = GAIN_TABLE_DISTANCES_IN;
        if (n == 1 || distanceIn <= d[0]) return values[0];
        for (int i = 1; i < n; i++) {
            if (distanceIn <= d[i]) {
                double span = d[i] - d[i - 1];
                double t = span > 0 ? (distanceIn - d[i - 1]) / span : 1.0;
                return values[i - 1] + t * (values[i] - values[i - 1]);
            }
        }
        return values[n - 1];
    }

    private void runFlywheel(double targetRPM) {
        this.targetRPM = targetRPM;

        if (targetRPM <= 0) {
            flywheelPID.reset();
            inBangBang = false;
            setFlywheelPower(0);
            return;
        }

        double targetTPS = rpmToTps(targetRPM);
        double currentTPS = getFlywheelVelocityTPS();
        double errorRPM = targetRPM - tpsToRpm(currentTPS);

        double power;
        if (FLYWHEEL_BANG_BANG_ENABLED && errorRPM > FLYWHEEL_BANG_BANG_THRESHOLD_RPM) {
            inBangBang = true;
            flywheelPID.reset(); // don't let I wind up while saturated
            power = 1.0;
        } else {
            inBangBang = false;
            double ff = flywheelFF.calculate(targetTPS);
            if (FLYWHEEL_VOLTAGE_COMPENSATION) {
                ff *= NOMINAL_VOLTAGE / getBatteryVoltage();
            }
            power = ff + flywheelPID.calculate(currentTPS, targetTPS);
        }

        setFlywheelPower(Math.max(0.0, Math.min(1.0, power)));
    }

    private void setFlywheelPower(double power) {
        lastFlywheelPower = power;
        if (!SKIP_FLYWHEEL) {
            leftLaunchMotor.set(power);
            rightLaunchMotor.set(power);
        }
    }

    /**
     * TEST ONLY (FlywheelEncoderTest): raw power to each flywheel motor separately, bypassing the
     * controller. Never use in match code: the two motors drive one flywheel and must match.
     */
    public void setFlywheelPowersForTest(double leftPower, double rightPower) {
        lastFlywheelPower = Math.max(leftPower, rightPower);
        leftLaunchMotor.set(leftPower);
        rightLaunchMotor.set(rightPower);
    }

    /** Loads the gains for this loop (picks up live Panels / FlywheelTuner edits too). */
    private void applyGains(double p, double i, double d, double ks, double kv) {
        activeP = p;
        activeI = i;
        activeD = d;
        flywheelPID.setPID(p, i, d);
        if (ffKs != ks || ffKv != kv) {
            ffKs = ks;
            ffKv = kv;
            flywheelFF = new SimpleMotorFeedforward(ffKs, ffKv);
        }
    }

    /** Gains used on the last flywheel update, as {P, I, D, kS, kV}. */
    public double[] getActiveFlywheelGains() {
        return new double[]{activeP, activeI, activeD, ffKs, ffKv};
    }

    /** Battery voltage, sampled every VOLTAGE_SAMPLE_MS. Falls back to NOMINAL_VOLTAGE if unavailable. */
    public double getBatteryVoltage() {
        long now = System.nanoTime() / 1_000_000L;
        if (batteryVoltageSensor != null && now - lastVoltageSampleMs >= VOLTAGE_SAMPLE_MS) {
            double v = batteryVoltageSensor.getVoltage();
            if (v > 6.0) batteryVoltage = v; // ignore bogus 0 readings
            lastVoltageSampleMs = now;
        }
        return batteryVoltage;
    }

    public double getLastFlywheelPower() {
        return lastFlywheelPower;
    }

    public boolean isFlywheelInBangBang() {
        return inBangBang;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isFlywheelAtSpeed() {
        return targetRPM > 0 && Math.abs(getCurrentFlywheelRPM() - targetRPM) < TARGET_RPM_TOLERANCE;
    }

    public static double rpmToTps(double rpm) {
        return rpm * Motor.GoBILDA.BARE.getCPR() / 60.0;
    }

    public static double tpsToRpm(double tps) {
        return tps * 60.0 / Motor.GoBILDA.BARE.getCPR();
    }


    public void setAlignmentLightColor(double color) {
        alignmentIndicatorLight.setPosition(color);
    }

    public void setTurretPosition(double position) {
        this.targetTurretPos = position;
        turretServo.setPosition(position);
        CrossOpModeStorage.turretPosition = position;
    }

    public double getTurretPosition() {
        return turretServo.getPosition();
    }

    /** Sets the flywheel PID gains (same as editing FLYWHEEL_P/I/D). */
    public void setFlywheelPID(double p, double i, double d) {
        FLYWHEEL_P = p;
        FLYWHEEL_I = i;
        FLYWHEEL_D = d;
    }

    /** Sets the flywheel feedforward gains (same as editing FLYWHEEL_KS/KV). */
    public void updateFeedforward(double ks, double kv) {
        FLYWHEEL_KS = ks;
        FLYWHEEL_KV = kv;
    }

    /*
    * Spins up the flywheel to the specified RPM. Converts to TPS internally
    * */
    public void spinUpFlywheelToRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        double ticksPerSecond = (targetRPM * Motor.GoBILDA.BARE.getCPR()) / 60.0;
        leftLaunchMotor.setVelocity(ticksPerSecond);
        rightLaunchMotor.setVelocity(ticksPerSecond);
    }

    public double getFlywheelVelocityTPS() {
        return (leftLaunchMotor.getVelocity() + rightLaunchMotor.getVelocity()) / 2.0;
    }
    public double getCurrentFlywheelRPM() {
        return tpsToRpm(getFlywheelVelocityTPS());
    }

    /** Per-motor RPM: if one reads ~0 while the other is at speed, check that encoder cable. */
    public double getLeftFlywheelRPM() {
        return tpsToRpm(leftLaunchMotor.getVelocity());
    }

    public double getRightFlywheelRPM() {
        return tpsToRpm(rightLaunchMotor.getVelocity());
    }

    public void setShotSolutionReady(boolean shotSolutionReady) {
        this.shotSolutionReady = shotSolutionReady;
    }

    public boolean isShotSolutionReady() {
        return shotSolutionReady;
    }

    /** False when no shot is possible from the current position (out of range / turret can't reach). */
    public void setShotPossible(boolean shotPossible) {
        this.shotPossible = shotPossible;
    }

    public boolean isShotPossible() {
        return shotPossible;
    }

    public void stop() {
        this.targetRPM = 0;
        flywheelPID.reset();
        inBangBang = false;
        setFlywheelPower(0);
    }

    /**
     * Checks if the flywheel and turret are both within acceptable tolerances
     * to ensure a successful shot.
     *
     * @return true if all systems are ready
     */
    public boolean isReadyToLaunch() {
        // 1. Check Flywheel RPM
        boolean flywheelReady = isFlywheelAtSpeed();

        // 2. Check Turret Alignment
        // Servos move fast, but we check if it has reached the target
        // A small tolerance is used for servo positioning
        boolean turretReady = Math.abs(getTurretPosition() - targetTurretPos) < TURRET_POSITION_TOLERANCE;

        return flywheelReady && turretReady && shotSolutionReady;
    }

    /*
     * This method is called periodically by the CommandScheduler. Useful for updating subsystem-specific state that you don't want to offload to a Command.
     * Teams should try to be consistent within their own codebases about which responsibilities will be handled by Commands, and which will be handled here.
     * Excellent for sending telemetry or updating dashboards, but optional!
     * */
    @Override
    public void periodic() {
//        if (targetRPM > 0) {
//            updateFlywheel(targetRPM);
//        } else {
//            stop();
//        }

        //handle the alignment indicator light
        if (!shotPossible)
            setAlignmentLightColor(ROBOT_ALIGNMENT_NOT_POSSIBLE_LIGHT); // reposition, waiting won't help
        else if (isReadyToLaunch())
            setAlignmentLightColor(ROBOT_ALIGNED_TO_SHOOT_LIGHT);
        else
            setAlignmentLightColor(ROBOT_NOT_ALIGNED_TO_SHOOT_LIGHT);

        // (You would pass a telemetry object into the subsystem constructor to use this)
        // telemetry.addData("Launcher Ready", isReady);
        // telemetry.addData("Current Speed", getCurrentVelocity());
        telemetry.addData("Flywheel Target RPM: ", targetRPM);
        telemetry.addData("Flywheel gains P/I/D/kS/kV", String.format("%.4f / %.4f / %.4f / %.3f / %.6f",
                activeP, activeI, activeD, ffKs, ffKv));
        if (USE_GAIN_TABLE && gainTableRows() < GAIN_TABLE_DISTANCES_IN.length) {
            telemetry.addData("Flywheel gain table", "WARNING: GAIN_TABLE_* arrays differ in length; extra rows ignored");
        }
    }
}
