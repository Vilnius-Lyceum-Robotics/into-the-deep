package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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

    private static Pose startingPose = new Pose(0, 0, Math.toRadians(0));
    public static Point lastRobotPosition = new Point(startingPose);

    @Override
    public void run() {

        GlobalConfig.DEBUG_MODE = true;

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);

        follower.setMaxPower(0.1);

        waitForStart();
        schedulePath();

        while (opModeIsActive()) {
            if (GlobalConfig.DEBUG_MODE) follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
        }
    }


    private void schedulePath() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new FollowPath(follower, 0, new Point(10, 0)),
                new WaitCommand(5000),
                new FollowPath(follower, 0, new Point(0, 0))
        ));
    }

    public static void setLastRobotPosition(Point endPoint) {lastRobotPosition = endPoint;}

    public static Point getLastRobotPosition() {return lastRobotPosition;}
}
