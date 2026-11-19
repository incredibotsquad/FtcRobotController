package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Locale;

/**
 * Runs the intake motor and the transfer motor separately, to isolate a rattle. Nothing else on the
 * robot is commanded (no launcher, gate, turret, drive or ball counting), and nothing moves in INIT.
 * Motors are driven directly with the same directions IntakeSubsystem uses (both inverted, so
 * positive = intaking / feeding up). Run it with no balls loaded.
 *
 * Gamepad 1:
 *   A            intake on / off
 *   B            transfer on / off
 *   X            reverse the direction of whichever motors are on
 *   Dpad up/down intake power +/- 0.1
 *   Dpad R/L     transfer power +/- 0.1
 *   Y            stop both
 * The Driver Station shows each motor's power, speed (ticks/s) and current (A).
 */
@TeleOp(name = "Test: Intake/Transfer Rattle", group = "Tests")
public class IntakeTransferRattleTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx transfer = hardwareMap.get(DcMotorEx.class, "transferMotor");
        for (DcMotorEx m : new DcMotorEx[]{intake, transfer}) {
            m.setDirection(DcMotorSimple.Direction.REVERSE); // = IntakeSubsystem's setInverted(true)
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast, so stopping doesn't jolt
        }

        telemetry.addLine("Intake/Transfer rattle test. Nothing moves until START.");
        telemetry.addLine("A = intake on/off, B = transfer on/off, X = reverse, Y = stop both");
        telemetry.addLine("Dpad up/down = intake power, Dpad right/left = transfer power");
        telemetry.update();
        waitForStart();

        boolean intakeOn = false, transferOn = false;
        double intakePower = 1.0, transferPower = 1.0;
        int direction = 1;

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) intakeOn = !intakeOn;
            if (gamepad1.bWasPressed()) transferOn = !transferOn;
            if (gamepad1.xWasPressed()) direction = -direction;
            if (gamepad1.yWasPressed()) { intakeOn = false; transferOn = false; }
            if (gamepad1.dpadUpWasPressed()) intakePower = clamp(intakePower + 0.1);
            if (gamepad1.dpadDownWasPressed()) intakePower = clamp(intakePower - 0.1);
            if (gamepad1.dpadRightWasPressed()) transferPower = clamp(transferPower + 0.1);
            if (gamepad1.dpadLeftWasPressed()) transferPower = clamp(transferPower - 0.1);

            intake.setPower(intakeOn ? direction * intakePower : 0);
            transfer.setPower(transferOn ? direction * transferPower : 0);

            telemetry.addData("Direction", direction > 0 ? "FORWARD (intaking / feeding up)" : "REVERSE");
            telemetry.addData("Intake (A)", line(intakeOn, intakePower, intake));
            telemetry.addData("Transfer (B)", line(transferOn, transferPower, transfer));
            telemetry.addLine("Run one at a time; listen, then change power to see if the rattle follows speed.");
            telemetry.update();
        }
        intake.setPower(0);
        transfer.setPower(0);
    }

    private static String line(boolean on, double power, DcMotorEx m) {
        return String.format(Locale.US, "%s  power %.1f  |  %.0f ticks/s  |  %.2f A",
                on ? "ON " : "off", power, m.getVelocity(), m.getCurrent(CurrentUnit.AMPS));
    }

    private static double clamp(double p) {
        return Math.round(Math.max(0.1, Math.min(1.0, p)) * 10) / 10.0;
    }
}
