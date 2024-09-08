package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.PrimaryDriverControls;
import org.firstinspires.ftc.teamcode.controls.SecondaryDriverControls;
import org.firstinspires.ftc.teamcode.helpers.commands.CommandRunner;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * @noinspection unchecked
 */
@Photon
@TeleOp(name = "VLRTeleOp")
public abstract class VLRTeleOp extends VLRLinearOpMode {
    // Execution
    ExecutorService executorService;
    // Commands
    CommandRunner commandRunner;
    // Controls
    PrimaryDriverControls primaryDriver;
    SecondaryDriverControls secondaryDriver;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(Claw.class, Lift.class);
        VLRSubsystem.initializeAll(hardwareMap);

        executorService = Executors.newCachedThreadPool();

        commandRunner = new CommandRunner(this::opModeIsActive);
        executorService.submit(commandRunner);

        primaryDriver = new PrimaryDriverControls(gamepad1);
        secondaryDriver = new SecondaryDriverControls(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            primaryDriver.update();
            secondaryDriver.update();
        }
    }
}
