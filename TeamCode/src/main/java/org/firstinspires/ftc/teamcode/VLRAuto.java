package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

@Config
@Photon
@Autonomous(name = "VLRAuto")
public class VLRAuto extends VLRLinearOpMode {
    private Follower follower;

    public static Pose startingPose = new Pose(9.6, 65, Math.toRadians(-180));
    private static Point lastRobotPosition = new Point(startingPose.getX(), startingPose.getY());

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(Chassis.class);
        VLRSubsystem.initializeAll(hardwareMap);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);

        waitForStart();
        schedulePath();

        while (opModeIsActive()) {
            follower.update();
            if (GlobalConfig.DEBUG_MODE) follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
        }
    }


    private void schedulePath() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new FollowPath(follower, -180, new Point(29, 65)),
                new FollowPath(follower, -180, 105, new Point(29, 50)),
                new FollowPath(follower, true, new Point(32.709, 36.206), new Point(44.846, 35.177), new Point(62.3, 35)),
                new FollowPath(follower, 180, new Point(62.3, 23)),
                new FollowPath(follower, 180, new Point(18, 23)),
                new FollowPath(follower, 180, new Point(62.3, 23)),
                new FollowPath(follower, 180, new Point(62.3, 13)),
                new FollowPath(follower, 180, new Point(20, 13)),
                new FollowPath(follower, 180, new Point(62.3, 13)),
                new FollowPath(follower, 180, new Point(62.3, 8.5)),
                new FollowPath(follower, 180, new Point(20, 8.5)),
                new FollowPath(follower, 180, new Point(12, 32)),
                new FollowPath(follower, true, new Point(25.303, 32.091), new Point(29.006, 37.851), new Point(29, 50)),
                new FollowPath(follower, -90, 180, new Point(29, 65))
        ));
    }

    public static void setLastRobotPosition(Point endPoint) {lastRobotPosition = endPoint;}

    public static Point getLastRobotPosition() {return lastRobotPosition;}


//    private BezierLine bezierLine (double startX, double startY, double endX, double endY) {return new BezierLine(new Point(startX, startY), new Point(endX, endY));}
//
//    private BezierCurve bezierCurve(Point... controlPoints) {return new BezierCurve(controlPoints);}
//
//    private Point point(double x, double y) {
//        return new Point(new Pose(x, y, Point.CARTESIAN));
//    }
}
