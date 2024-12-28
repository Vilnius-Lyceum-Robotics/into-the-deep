package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroCommands.HoldPoint;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;

@Config
@Photon
@Autonomous(name = "VLRAuto")
public class VLRAuto extends VLRLinearOpMode {
    private Follower follower;

    private static Pose startingPose = new Pose(9.6, 63.2, Math.toRadians(0));
    public static Point lastRobotPosition = new Point(startingPose);

    @Override
    public void run() {

        GlobalConfig.DEBUG_MODE = true;

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);

        follower.setMaxPower(0.6);

        waitForStart();
        schedulePath();
        CommandScheduler.getInstance().run();
        while (opModeIsActive()) {
            if (GlobalConfig.DEBUG_MODE){
                if(follower.getPose() != null){
                    follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
                }
            }
        }
    }


    private void schedulePath() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new FollowPath(follower, 0, new Point(9.6, 20))
//                new FollowPath(follower, 0, new Point(28, 63.2)),
//                new FollowPath(follower, 0, 90, new Point(23.2, 63.2)),
//                new FollowPath(follower, true,
//                        new Point(38.47, 30.1), new Point(22.96, 32), new Point(52, 30)),
//                new FollowPath(follower, 0, new Point(52, 28)),
//                new FollowPath(follower, 0, new Point(6, 28)),
//                new HoldPoint(follower, new Pose(6, 28, 0))

//                new FollowPath(follower, 180, new Point(62.3, 23)),
//                new FollowPath(follower, 180, new Point(62.3, 13)),
//                new FollowPath(follower, 180, new Point(20, 13)),
//                new FollowPath(follower, 180, new Point(62.3, 13)),
//                new FollowPath(follower, 180, new Point(62.3, 8.5)),
//                new FollowPath(follower, 180, new Point(20, 8.5)),
//                new FollowPath(follower, 180, new Point(12, 32)),
//
//                new FollowPath(follower, true,
//                        new Point(25.303, 32.091), new Point(29.006, 37.851), new Point(29, 50)),
//
//                new FollowPath(follower, -90, 180, new Point(29, 63.2))
        ));
    }

    public static void setLastRobotPosition(Point endPoint) {lastRobotPosition = endPoint;}

    public static Point getLastRobotPosition() {return lastRobotPosition;}
}
