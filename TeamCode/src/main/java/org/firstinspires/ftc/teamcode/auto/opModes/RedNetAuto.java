package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commandFactory.NetCommandFactory;
import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

@Config
@Photon
@Autonomous(name = "RedNetAuto", group = "Red Team")
public class RedNetAuto extends VLRLinearOpMode {

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);
        GlobalConfig.setIsInvertedMotors(true);
        NetCommandFactory commandFactory = new NetCommandFactory(false);
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(commandFactory.getStartingPoint(), 0));
        follower.setMaxPower(0.6);

        FollowPath.setStartingPoint(commandFactory.getStartingPoint());
        FollowPath.setFollower(follower);

        waitForStart();
        CommandScheduler.getInstance().schedule(commandFactory.getCommands());
//        CommandScheduler.getInstance().run();

        while (opModeIsActive()) {
//            follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
        }
    }
}
