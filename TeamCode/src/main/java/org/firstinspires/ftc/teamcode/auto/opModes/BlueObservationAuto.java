package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.pathFactory.ObservationPathFactory;
import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;

@Config
@Photon
@Autonomous(name = "BlueObservationAuto", group = "Blue Team")
public class BlueObservationAuto extends VLRLinearOpMode {

    @Override
    public void run() {
        GlobalConfig.setIsInvertedMotors(true);

        ObservationPathFactory observationPath = new ObservationPathFactory(true);
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(observationPath.getStartingPoint(), 0));
        follower.setMaxPower(0.6);

        FollowPath.setStartingPoint(observationPath.getStartingPoint());
        FollowPath.setFollower(follower);

        waitForStart();
        CommandScheduler.getInstance().schedule(observationPath.getPathCommand());
        CommandScheduler.getInstance().run();

        while (opModeIsActive()) {
            follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
        }

    }
}
