package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;

@Photon
@Autonomous(name = "VLRAuto")
public class VLRAuto extends VLRLinearOpMode {
    Follower follower;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(Chassis.class);
        VLRSubsystem.initializeAll(hardwareMap);

        follower = new Follower(hardwareMap);

        Pose startingPose = new Pose(9.6, 65, Math.toRadians(-180)); // todo cfg
        follower.setStartingPose(startingPose);

        waitForStart();
        schedulePath();

        while (opModeIsActive()) {
            follower.update();
            if (GlobalConfig.DEBUG_MODE){
                follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
            }
        }
    }


    private void schedulePath() {
        PathBuilder builder = follower.pathBuilder();

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new FollowPath(follower, builder.addPath(bezierLine(9.6, 65, 29, 65))
                        .setConstantHeadingInterpolation(Math.toRadians(-180)).build()),

                new FollowPath(follower, builder.addPath(bezierLine(29, 65, 29, 50))
                        .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(105)).build()),

                new FollowPath(follower, builder.addPath(bezierCurve(point(29, 50), point(32.709, 36.206), point(44.846, 35.177), point(62.3, 35)))
                        .setTangentHeadingInterpolation().setReversed(true).build())),

                new FollowPath(follower, builder.addPath(bezierLine(62.3, 35, 62.3, 23))
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()),

                new FollowPath(follower, builder.addPath(bezierLine(62.3, 23, 18, 23))
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()),

                new FollowPath(follower, builder.addPath(bezierLine(18, 23, 62.3, 23))
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()),

                new FollowPath(follower, builder.addPath(bezierLine(62.3, 23, 62.3, 13))
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()),

                new FollowPath(follower, builder.addPath(bezierLine(62.3, 13, 20, 13))
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()),

                new FollowPath(follower, builder.addPath(bezierLine(20, 13, 62.3, 13))
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()),

                new FollowPath(follower, builder.addPath(bezierLine(62.3, 13, 62.3, 8.5))
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()),

                new FollowPath(follower, builder.addPath(bezierLine(62.3, 8.5, 20, 8.5))
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()),

                new FollowPath(follower, builder.addPath(bezierLine(20, 8.5, 12, 32))
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()),

                new FollowPath(follower, builder.addPath(bezierCurve(point(12, 32), point(25.303, 32.091), point(29.006, 37.851), point(29, 50)))
                        .setTangentHeadingInterpolation().setReversed(true).build()),

                new FollowPath(follower, builder.addPath(bezierLine(29, 50, 29, 65))
                        .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180)).build())
        );
    }

    private BezierLine bezierLine (double startX, double startY, double endX, double endY) {return new BezierLine(new Point(startX, startY), new Point(endX, endY));}

    private BezierCurve bezierCurve(Point... controlPoints) {return new BezierCurve(controlPoints);}

    private Point point(double x, double y) {
        return new Point(new Pose(x, y, Point.CARTESIAN));
    }
}
