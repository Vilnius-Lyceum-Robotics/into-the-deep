package org.firstinspires.ftc.teamcode.helpers.utils;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotator;

@Photon
@TeleOp(name = "ArmRotatorMotionProfileTest")
public class ArmRotatorMotionProfileTest extends VLRLinearOpMode {
    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(ArmRotator.class);
        VLRSubsystem.initializeAll(hardwareMap);

        ArmRotator armRotator = VLRSubsystem.getInstance(ArmRotator.class);

        waitForStart();

        DriverControls dc = new DriverControls(new GamepadEx(gamepad1));

        dc.add(new ButtonCtl(GamepadKeys.Button.A, ButtonCtl.Trigger.WAS_JUST_PRESSED, (Boolean a) -> armRotator.setTargetPos(ArmRotator.TargetPosition.UP)));
        dc.add(new ButtonCtl(GamepadKeys.Button.B, ButtonCtl.Trigger.WAS_JUST_PRESSED, (Boolean b) -> armRotator.setTargetPos(ArmRotator.TargetPosition.DOWN)));

        while (opModeIsActive()) {
            dc.update();
        }
    }
}
