package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.PrimaryDriverTeleOpControls;
import org.firstinspires.ftc.teamcode.controls.SecondaryDriverTeleOpControls;
import org.firstinspires.ftc.teamcode.helpers.commands.CommandRunner;
import org.firstinspires.ftc.teamcode.helpers.monitoring.LoopTimeMonitor;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pose2D;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * @noinspection unchecked
 */
@Photon
@TeleOp(name = "VLRTeleOp")
public class VLRTeleOp extends VLRLinearOpMode {
    // Execution
    ExecutorService executorService;
    // Commands
    CommandRunner commandRunner;
    // Controls
    PrimaryDriverTeleOpControls primaryDriver;
    SecondaryDriverTeleOpControls secondaryDriver;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(Chassis.class, Pinpoint.class);
        VLRSubsystem.initializeAll(hardwareMap);

        executorService = Executors.newCachedThreadPool();

        commandRunner = new CommandRunner(this::opModeIsActive);
        executorService.submit(commandRunner);

        primaryDriver = new PrimaryDriverTeleOpControls(gamepad1);
        secondaryDriver = new SecondaryDriverTeleOpControls(gamepad2);
//
//        LoopTimeMonitor ltm = new LoopTimeMonitor();

        Pinpoint pinpoint = VLRSubsystem.getInstance(Pinpoint.class);

        waitForStart();

        while (opModeIsActive()) {
            primaryDriver.update();
            secondaryDriver.update();

            Pose2D pose = pinpoint.getPose();
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getHeading());
            telemetry.update();
        }
    }
}
