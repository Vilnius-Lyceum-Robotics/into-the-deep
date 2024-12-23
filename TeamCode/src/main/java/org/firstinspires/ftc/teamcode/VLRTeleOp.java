package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.PrimaryDriverTeleOpControls;
import org.firstinspires.ftc.teamcode.controls.SecondaryDriverTeleOpControls;
import org.firstinspires.ftc.teamcode.helpers.monitoring.LoopTimeMonitor;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pose2D;

import java.util.ArrayList;
import java.util.List;

/**
 * @noinspection unchecked
 */
@Photon
@TeleOp(name = "VLRTeleOp")
public class VLRTeleOp extends VLRLinearOpMode {
    // Controls
    PrimaryDriverTeleOpControls primaryDriver;
    SecondaryDriverTeleOpControls secondaryDriver;

    List<LynxModule> allHubs;
    double loopTime = 0;

    @Override
    public void run() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        VLRSubsystem.requireSubsystems(Chassis.class, Pinpoint.class, ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        primaryDriver = new PrimaryDriverTeleOpControls(gamepad1);
        secondaryDriver = new SecondaryDriverTeleOpControls(gamepad2);

        Pinpoint pinpoint = VLRSubsystem.getInstance(Pinpoint.class);

        GlobalConfig.DEBUG_MODE = false;
        ArmRotatorSubsystem arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

        while (opModeIsActive()) {
            clearBulkCache();

            primaryDriver.update();
            secondaryDriver.update();

            Pose2D pose = pinpoint.getPose();

            double loop = System.nanoTime();
            double dt = (loop - loopTime) / 1000000;
            telemetry.addData("Loop time ms", dt);
            telemetry.addData("Loop time hz", 1000 / dt);
            loopTime = loop;

            if (GlobalConfig.DEBUG_MODE) {
                telemetry.addData("X", pose.getX());
                telemetry.addData("Y", pose.getY());
                telemetry.addData("Heading", pose.getHeading());
                telemetry.addData("current state", arm.getArmState());
                telemetry.update();
            }
        }
    }


    public void setBulkCachingMode(LynxModule.BulkCachingMode mode){
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(mode);
        }
    }

    public void clearBulkCache(){
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
}
