package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration;

@Photon
@Config
@TeleOp(name = "ArmMotionProfileTuningWSubsystem")

public class SubsystemTest extends VLRLinearOpMode {
    ArmSubsystem armSubsystem;
    public static ArmConfiguration.TargetAngle targetAngle = ArmConfiguration.TargetAngle.DOWN;
    public static SlideConfiguration.TargetPosition targetPosition = SlideConfiguration.TargetPosition.RETRACTED;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(ArmSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        armSubsystem = VLRSubsystem.getInstance(ArmSubsystem.class);

        waitForStart();

        while(opModeIsActive()){
            armSubsystem.setTargetAngle(targetAngle);
            armSubsystem.setTargetPosition(targetPosition);
        }
    }
}