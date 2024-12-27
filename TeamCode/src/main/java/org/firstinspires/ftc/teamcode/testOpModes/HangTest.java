package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;

@Photon
@Config
@TeleOp(name = "HangTest")

public class HangTest extends VLRLinearOpMode {

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(HangSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        HangSubsystem hang = VLRSubsystem.getInstance(HangSubsystem.class);

        GlobalConfig.DEBUG_MODE = true;
        GlobalConfig.PRINT_MOTOR_CURRENT = true;

        waitForStart();

        while(opModeIsActive()){
            hang.setPower(gamepad1.right_stick_y);
        }
    }
}