package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetArmState_Deposit;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetArmState_InRobot;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetArmState_Intake;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

/**
 * Abstraction for secondary driver controls. All controls will be defined here.
 * For this to work well, all subsystems will be defined as singletons.
 */
public class SecondaryDriverTeleOpControls extends DriverControls {
    ClawSubsystem claw;
    ArmRotatorSubsystem arm;
    ArmSlideSubsystem slide;
    CommandScheduler cs;

    GamepadKeys.Button TRIANGLE = GamepadKeys.Button.Y;
    GamepadKeys.Button SQUARE = GamepadKeys.Button.X;
    GamepadKeys.Button CROSS = GamepadKeys.Button.A;
    GamepadKeys.Button CIRCLE = GamepadKeys.Button.B;

    public SecondaryDriverTeleOpControls(Gamepad gamepad) {
        super(new GamepadEx(gamepad));

        claw = VLRSubsystem.getInstance(ClawSubsystem.class);
        arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
        slide = VLRSubsystem.getInstance(ArmSlideSubsystem.class);
        cs = CommandScheduler.getInstance();

        // Define controls here
//        add(new ButtonCtl(GamepadKeys.Button.A, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean a) -> cs.schedule(new ClawGrabFromSubmersibleCommand())));
//        add(new ButtonCtl(GamepadKeys.Button.B, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean b) -> cs.schedule(new ArmExtensionCommand())));
//        add(new ButtonCtl(GamepadKeys.Button.X, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean b) -> cs.schedule(new ClawAngleSubmersibleCommand())));
//        add(new ButtonCtl(GamepadKeys.Button.Y, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean b) -> cs.schedule(new ArmDepositCommand())));

        add(new ButtonCtl(CROSS, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean a) -> cs.schedule(new SetArmState_Intake())));
        add(new ButtonCtl(SQUARE, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean b) -> cs.schedule(new SetArmState_InRobot())));
        add(new ButtonCtl(TRIANGLE, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean c) -> cs.schedule(new SetArmState_Deposit())));


//        add(new ButtonCtl(CustomGamepadKeys.CIRCLE, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean b) -> cs.schedule(new ArmExtensionCommand())));
//        add(new ButtonCtl(CustomGamepadKeys.TRIANGLE, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean b) -> cs.schedule(new ClawAngleSubmersibleCommand())));
//        add(new ButtonCtl(CustomGamepadKeys.SQUARE, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean b) -> cs.schedule(new ArmDepositCommand())));


        //addRightStickHandler((Double x, Double y) -> clawRotationRightStick(y));
        //addLeftStickHandler((Double x, Double y) -> liftLeftStick(x));
    }

//    private void clawRotationRightStick(double y) {
//        if (arm.getRotatorState() == ArmRotatorConfiguration.RotatorState.PRE_INTAKE) {
//            claw.setTwistIncrement(y * 0.02);
//        }
//    }
//
//    private void liftLeftStick(double x) {
//        if (arm.getRotatorState() == ArmRotatorConfiguration.RotatorState.PRE_INTAKE) {
//            slide.incrementTargetPosition(x * 0.5);
//        }
//    }
}
