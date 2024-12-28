package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;

@Config
@Photon
@Autonomous(name = "AutoPointTuner")
public class AutoPointTuner extends VLRLinearOpMode {
    private Follower follower;

    public static double targetX = 10;
    public static double targetY = 63.2;
    public static double targetHeading = -180;

    private Pose targetPose = new Pose(targetX, targetY, Math.toRadians(targetHeading));
    private Pose prevPose = targetPose;

    @Override
    public void run() {
        GlobalConfig.DEBUG_MODE = true;

        follower = new Follower(hardwareMap);
        follower.setStartingPose(targetPose);

        follower.setMaxPower(0.1);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            targetPose = new Pose(targetX, targetY, Math.toRadians(targetHeading));

            if (!prevPose.roughlyEquals(targetPose)){

                Pose currentPose = follower.getPose();

                CommandScheduler.getInstance().schedule(
                        new FollowPath(follower, follower.pathBuilder().addPath(new BezierLine(new Point(currentPose), new Point(targetPose)))
                                .setConstantHeadingInterpolation(targetPose.getHeading()).build())
                );

                prevPose = targetPose.copy();
            }

            if (GlobalConfig.DEBUG_MODE) follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
        }
    }
}
