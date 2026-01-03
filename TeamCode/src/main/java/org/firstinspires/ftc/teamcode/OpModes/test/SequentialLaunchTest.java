package org.firstinspires.ftc.teamcode.OpModes.test;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.LaunchKickAction;
import org.firstinspires.ftc.teamcode.Actions.SpindexAction;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

import java.util.ArrayList;
import java.util.List;

//@Disabled

@Config
@TeleOp(name="SequentialLaunchTest", group="Tests")
public class SequentialLaunchTest extends LinearOpMode {

    // Declare OpMode members.
    public static String servo1Name = "SpindexServo";
    public static String servo2Name = "LaunchKickServo";
    private RobotHardware robotHardware;

    private List<Action> runningActions;
    private FtcDashboard dashboard;

    // Close: 0.42
    // Open: 0.55
    private ElapsedTime runtime = new ElapsedTime();
    private Servo Servo1;
    private Servo Servo2;


    @Override
    public void runOpMode() {

        runningActions = new ArrayList<>();
        dashboard = FtcDashboard.getInstance();
        robotHardware = new RobotHardware(this.hardwareMap);

        while (opModeInInit()) {

        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.aWasPressed()) {
                Log.i("SequentialLaunchTest", "Adding full launch sequence");

                runningActions.add(
                        new SequentialAction(
                                new InstantAction(() -> Log.i("SequentialLaunchTest", "Starting actions")),
                                new InstantAction(() -> runtime.reset()),
                                new SpindexAction(robotHardware, Spindex.LAUNCH_POS_1),
                                new LaunchKickAction(robotHardware),
                                new SpindexAction(robotHardware, Spindex.LAUNCH_POS_3),
                                new LaunchKickAction(robotHardware),
                                new SpindexAction(robotHardware, Spindex.LAUNCH_POS_2),
                                new LaunchKickAction(robotHardware),
                                new InstantAction(() -> Log.i("SequentialLaunchTest", "Total time: " + runtime.milliseconds()))
                        )
                );
            }

            if (gamepad1.bWasPressed()) {
                Log.i("SequentialLaunchTest", "Adding kick only");
                runningActions.add(
                        new SequentialAction(
                                new SpindexAction(robotHardware, Spindex.LAUNCH_POS_3)
                        )
                );
            }

            ProcessActions();
        }
    }

    private void ProcessActions()
    {
        TelemetryPacket packet = new TelemetryPacket();

        // we have actions to run, state transition in progress
        if (!runningActions.isEmpty()) {

            // run actions and add pending ones to new list
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());

                if (action.run(packet)) {
                    newActions.add(action); //add if action indicates it needs to run again
                }
            }
            dashboard.sendTelemetryPacket(packet);
            runningActions = newActions;
        }
    }
}
