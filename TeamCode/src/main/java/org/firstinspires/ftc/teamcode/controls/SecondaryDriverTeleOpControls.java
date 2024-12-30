package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.MoveArmToDeposit;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.MoveArmInToRobot;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.MoveArmToIntake;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
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

        add(new ButtonCtl(CROSS, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean a) -> cs.schedule(new MoveArmToIntake())));
        add(new ButtonCtl(SQUARE, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean b) -> cs.schedule(new MoveArmInToRobot())));
        add(new ButtonCtl(TRIANGLE, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean c) -> cs.schedule(new MoveArmToDeposit())));

        add(new ButtonCtl(GamepadKeys.Button.DPAD_UP, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean d) -> cs.schedule(new SetClawAngle(ClawConfiguration.TargetAngle.UP))));
        add(new ButtonCtl(GamepadKeys.Button.DPAD_DOWN, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean e) -> cs.schedule(new SetClawAngle(ClawConfiguration.TargetAngle.DOWN))));
        add(new ButtonCtl(GamepadKeys.Button.DPAD_LEFT, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean f) -> cs.schedule(new SetClawState(ClawConfiguration.TargetState.OPEN))));
        add(new ButtonCtl(GamepadKeys.Button.DPAD_RIGHT, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean g) -> cs.schedule(new SetClawState(ClawConfiguration.TargetState.CLOSED_NORMAL))));

        addRightStickHandler((Double x, Double y) -> incrementClaw(x));
        addLeftStickHandler((Double x, Double y) -> incrementSlidePosition(x));
    }

    private void incrementClaw(double input) {
        if (ArmState.get() == ArmState.State.INTAKE) {
            claw.setTwistIncrement(input * 0.08);
        }
    }

    private void incrementSlidePosition(double input) {
        if (ArmState.get() == ArmState.State.INTAKE) {
            slide.incrementTargetPosition(input * 0.2);
        }
    }
}
