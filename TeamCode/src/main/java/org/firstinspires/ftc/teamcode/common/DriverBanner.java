package org.firstinspires.ftc.teamcode.common;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * A large, colored message at the top of the Driver Station for a few seconds, e.g. "RELOCALIZED"
 * with the new pose, so drivers can confirm an action at a glance.
 *
 * Any code calls show(); the OpMode calls render(telemetry) every loop before its telemetry update.
 * render() switches the SDK telemetry to HTML (Html.fromHtml: big, b, font color), so the banner
 * renders large on the Driver Station. Other telemetry lines are plain text and show as before.
 */
@Configurable
public class DriverBanner {

    /** How long a banner stays on screen. */
    public static long DISPLAY_MS = 4000;

    private static volatile String title;
    private static volatile String detail;
    private static volatile String color = "#00E676";
    private static volatile long untilMs = 0;

    private DriverBanner() {}

    /** Shows a banner for DISPLAY_MS. {@code colorHex} like "#00E676"; null keeps green. */
    public static void show(String bannerTitle, String bannerDetail, String colorHex) {
        title = bannerTitle;
        detail = bannerDetail;
        color = colorHex != null ? colorHex : "#00E676";
        untilMs = System.currentTimeMillis() + DISPLAY_MS;
    }

    /** "RELOCALIZED" banner with the pose it reset to (inches, degrees). */
    public static void showRelocalized(String what, double x, double y, double headingDeg) {
        show("RELOCALIZED", String.format(Locale.US, "x %.1f&nbsp;&nbsp; y %.1f&nbsp;&nbsp; %.0f&deg;<br><small>%s</small>",
                x, y, headingDeg, what), "#00E676");
    }

    /** Adds the banner (if one is active) to the SDK telemetry. Call every loop before telemetry.update(). */
    public static void render(Telemetry telemetry) {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        if (System.currentTimeMillis() >= untilMs || title == null) return;
        telemetry.addLine(String.format("<big><big><big><b><font color=\"%s\">&#10004; %s</font></b></big></big></big>", color, title));
        if (detail != null) telemetry.addLine("<big><big><b>" + detail + "</b></big></big>");
        telemetry.addLine("&nbsp;");
    }
}
