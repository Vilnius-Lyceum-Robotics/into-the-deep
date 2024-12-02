package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmRotatingPartConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration;

@Photon
@Config
@TeleOp(name = "armMotionProfileTuningWSubsystem")

public class ArmSubsystemTest extends VLRLinearOpMode {
    ArmSubsystem armSubsystem;
    public static ArmRotatingPartConfiguration.TargetAngle targetAngle = ArmRotatingPartConfiguration.TargetAngle.DOWN;
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