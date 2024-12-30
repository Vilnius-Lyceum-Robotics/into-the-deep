package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetArmState_Deposit;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetArmState_InRobot;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

@Config
@Photon
@Autonomous(name = "VLRAuto_bucket")
public class VLRAuto_bucket extends VLRLinearOpMode {
    private Follower follower;

    private final double xStart = 10;
    private final double yStart = 111.5;

    @Override
    public void run() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(xStart, yStart, 0));
        follower.setMaxPower(0.6);

        FollowPath.setStartingPoint(new Point(xStart, yStart));
        FollowPath.setFollower(follower);

        waitForStart();
        schedulePath();

        while (opModeIsActive()) {
            //follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
            telemetry.addData("armState", ArmState.get());
            telemetry.update();
        }
    }

    private void schedulePath() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                //new FollowPath(0, -50, new Point(20, 120)),

                new SetArmState_Deposit(),

                new WaitCommand(500),

                new SetArmState_InRobot(telemetry),

                new WaitCommand(10000)
        ));
    }

}
