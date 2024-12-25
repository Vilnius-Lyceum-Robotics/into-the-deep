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

    public static Pose startingPose = new Pose(9.6, 65, Math.toRadians(-180));
    public static Pose targetPose = startingPose;

    private Pose prevPose = targetPose;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(Chassis.class);
        VLRSubsystem.initializeAll(hardwareMap);

        GlobalConfig.DEBUG_MODE = true;

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);

        waitForStart();

        while (opModeIsActive()) {

            if (prevPose != targetPose){
                Pose currentPose = follower.getPose();

                CommandScheduler.getInstance().schedule(
                        new FollowPath(follower, follower.pathBuilder().addPath(new BezierLine(new Point(currentPose), new Point(targetPose)))
                                .setConstantHeadingInterpolation(targetPose.getHeading()).build())
                );

                prevPose = targetPose;
            }

            follower.update();
            if (GlobalConfig.DEBUG_MODE) follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
        }
    }
}
