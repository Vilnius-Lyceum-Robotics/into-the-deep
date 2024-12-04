package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.ArmDepositCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.ArmExtensionCommand;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.ClawAngleSubmersibleCommand;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.ClawGrabFromSubmersibleCommand;

/**
 * Abstraction for secondary driver controls. All controls will be defined here.
 * For this to work well, all subsystems will be defined as singletons.
 */
public class SecondaryDriverTeleOpControls extends DriverControls {
    ClawSubsystem claw;
    ArmSubsystem arm;
    SlideSubsystem slide;
    CommandScheduler cs;

    public SecondaryDriverTeleOpControls(Gamepad gamepad) {
        super(new GamepadEx(gamepad));

        claw = VLRSubsystem.getInstance(ClawSubsystem.class);
        arm = VLRSubsystem.getInstance(ArmSubsystem.class);
        slide = VLRSubsystem.getInstance(SlideSubsystem.class);
        cs = CommandScheduler.getInstance();

        // Define controls here
        add(new ButtonCtl(GamepadKeys.Button.A, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean a) -> cs.schedule(new ClawGrabFromSubmersibleCommand())));
        add(new ButtonCtl(GamepadKeys.Button.B, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean b) -> cs.schedule(new ArmExtensionCommand())));
        add(new ButtonCtl(GamepadKeys.Button.X, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean b) -> cs.schedule(new ClawAngleSubmersibleCommand())));
        add(new ButtonCtl(GamepadKeys.Button.Y, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean b) -> cs.schedule(new ArmDepositCommand())));


        addRightStickHandler((Double x, Double y) -> clawRotationRightStick(y));
        addLeftStickHandler((Double x, Double y) -> liftLeftStick(x));
    }

    private void clawRotationRightStick(double y) {
        if (arm.getState() == ArmSubsystem.ArmState.PRE_INTAKE) {
            claw.incrementTwist(y * 0.02);
        }
    }

    private void liftLeftStick(double x) {
        if (arm.getState() == ArmSubsystem.ArmState.PRE_INTAKE) {
            slide.incrementTargetPosition(x * 0.5);
        }
    }
}
