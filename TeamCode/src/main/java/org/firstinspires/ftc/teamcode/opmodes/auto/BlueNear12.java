package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Incredibot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;

import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;
@Autonomous(name = "Blue Near 1+2", group = "Autonomous")
public class BlueNear12 extends CommandOpMode {

    private Incredibot robot;
    private Follower follower;

    // initialize Poses
    private final Pose startPose = new Pose(22, 122, Math.toRadians(324)); // Start Pose of our robot. This is against the goal facing AWAY
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(135)); // Scoring Pose of our robot.
    private final Pose pickup1Pose = new Pose(17, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(12, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(12, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose endPose = new Pose (60, 105); // Final Pose of our robot, off the starting line

    //defining our PathChains
    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, leave;

    public Command autoRoutine() {
        return sequential(
                /* Go To Score Command*/
                follow(follower, scorePreload),
                /* Collect 3 Artifacts Command*/
                follow(follower, grabPickup1, true),
                /* Go Back To Score Command*/
                follow(follower, scorePickup1, true),
                /* Collect 3 Artifacts Command*/
                follow(follower, grabPickup2, true),
                /* Go Back To Score Command*/
                follow(follower, scorePickup2, true),
                /* Collect 3 Artifacts Command*/
                follow(follower, grabPickup3, true),
                /* Go Back To Score Command*/
                follow(follower, scorePickup3, true),
                /* Leave Start Line Command*/
                follow(follower, leave, true)
        );
    }

    @Override
    public void initialize() {

        Scheduler.reset();

        // Setup Pedro Follower (This should be initialized in your OdometrySubsystem usually)
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        // Initialize the robot container
        // We pass 'null' for gamepads since it's Autonomous
        robot = new Incredibot(hardwareMap, Incredibot.OpModeType.AUTO, null, null, telemetry);

        Scheduler.schedule(autoRoutine());
    }

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierCurve (curved line). */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(60, 54), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierCurve (curved line). */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, new Pose(60, 54), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierCurve (curved line). */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(60, 30), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierCurve (curved line). */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose,new Pose(60, 30), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our leave PathChain. We are using a single path using a BezierLine (straight line).
         * We use Constant Interpolation here instead of Linear*/
        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }

    @Override
    public void run() {
        super.run();
        follower.update(); // Crucial for Pedro Pathing to calculate movements
        Scheduler.execute();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.update();
    }
}