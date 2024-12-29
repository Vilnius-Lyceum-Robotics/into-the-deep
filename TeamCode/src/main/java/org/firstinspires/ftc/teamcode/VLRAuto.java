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

    private static final Pose startingPose = new Pose(9.6, 60);
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
            follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
        }
    }


    private void schedulePath() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new FollowPath(follower, 0, new Point(9.6, 60), new Point(28, 60)),
                new WaitCommand(500),
                new FollowPath(follower, 0, -90, new Point(28, 60), new Point(28.05, 60)),
                new FollowPath(follower, false,
                        new Point(28, 60),
                        new Point(29, 42.5),
                        new Point(38.5, 29),
                        new Point(56, 29)),


                new FollowPath(follower, 0, new Point(56, 29), new Point(56, 20)),
                new FollowPath(follower, 0, new Point(56,20), new Point(21, 20))
//                new FollowPath(follower, 0, new Point(56, 20)),
//                new FollowPath(follower, 0, new Point(56, 17)),
//                new FollowPath(follower, 0, new Point(21, 17))





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
