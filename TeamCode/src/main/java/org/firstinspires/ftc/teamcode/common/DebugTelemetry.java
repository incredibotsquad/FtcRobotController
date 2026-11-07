package org.firstinspires.ftc.teamcode.common;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Switch for the subsystems' and commands' per-loop telemetry readouts (drive, odometry, intake,
 * intake bar, launcher, Limelight, launch readiness). Off by default: building those strings every
 * loop costs loop time and clutters the Driver Station. Turn on in Panels (pit only, R704) to debug.
 * Event logs (logcat) are unaffected; they only fire on changes.
 */
@Configurable
public final class DebugTelemetry {
    public static boolean ENABLED = false;

    private DebugTelemetry() {}
}
