package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;

@Config
@Photon
@Autonomous(name = "AutoPointTuner")
public class AutoPointTuner extends VLRLinearOpMode {
    private Follower follower;

    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetHeading = 0;

    private Pose targetPose = new Pose(targetX, targetY, Math.toRadians(targetHeading));
    private Pose prevPose = targetPose;

    @Override
    public void run() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(targetPose);

        follower.setMaxPower(0.4);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();
        CommandScheduler.getInstance().schedule(
                new FollowPath(follower, follower.pathBuilder().addPath(new BezierLine(new Point(0, 0), new Point(targetPose)))
                        .setConstantHeadingInterpolation(targetPose.getHeading()).build())
        );
        CommandScheduler.getInstance().run();


        while (opModeIsActive()) {
            follower.update();

            targetPose = new Pose(targetX, targetY, Math.toRadians(targetHeading));

            if (!prevPose.roughlyEquals(targetPose)){
                CommandScheduler.getInstance().schedule(
                        new FollowPath(follower, follower.pathBuilder().addPath(new BezierLine(new Point(prevPose), new Point(targetPose)))
                                .setConstantHeadingInterpolation(targetPose.getHeading()).build())
                );
                prevPose = new Pose(targetX, targetY, Math.toRadians(targetHeading));
            }

            follower.telemetryDebug(telemetry);
            telemetry.update();
        }
    }
}