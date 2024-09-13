package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.PrimaryDriverTeleOpControls;
import org.firstinspires.ftc.teamcode.controls.SecondaryDriverTeleOpControls;
import org.firstinspires.ftc.teamcode.helpers.commands.CommandRunner;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.starterClaw.StarterClaw;
import org.firstinspires.ftc.teamcode.subsystems.mainArm.MainArm;

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
        VLRSubsystem.requireSubsystems(StarterClaw.class, MainArm.class);
        VLRSubsystem.initializeAll(hardwareMap);

        executorService = Executors.newCachedThreadPool();

        commandRunner = new CommandRunner(this::opModeIsActive);
        executorService.submit(commandRunner);

        primaryDriver = new PrimaryDriverTeleOpControls(gamepad1);
        secondaryDriver = new SecondaryDriverTeleOpControls(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            primaryDriver.update();
            secondaryDriver.update();
        }
    }
}
