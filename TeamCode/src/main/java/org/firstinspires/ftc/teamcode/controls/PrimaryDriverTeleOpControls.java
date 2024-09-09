package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.subsystems.lift.commands.LiftRunToPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.lift.commands.LiftToggleCommand;
import org.firstinspires.ftc.teamcode.subsystems.testSingleMotor.commands.SingleMotorSetPower;

/**
 * Abstraction for primary driver controls. All controls will be defined here.
 * For this to work well, all subsystems will be defined as singletons.
 */
public class PrimaryDriverTeleOpControls extends DriverControls {
    public PrimaryDriverTeleOpControls(Gamepad gamepad) {
        super(new GamepadEx(gamepad));

        CommandScheduler cs = CommandScheduler.getInstance();

//        add(new ButtonCtl(GamepadKeys.Button.LEFT_BUMPER, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean lb) -> cs.schedule(new ClawToggleCommand())));
//        add(new ButtonCtl(GamepadKeys.Button.RIGHT_BUMPER, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean rb) -> cs.schedule(new LiftToggleCommand())));
//        add(new ButtonCtl(GamepadKeys.Button.A, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean a) -> cs.schedule(new LiftRunToPositionCommand(5000))));

        add(new ButtonCtl(GamepadKeys.Button.DPAD_UP, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean dpadUp) -> cs.schedule(new SingleMotorSetPower(1))));
        add(new ButtonCtl(GamepadKeys.Button.DPAD_DOWN, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean dpadDown) -> cs.schedule(new SingleMotorSetPower(-1))));
        add(new ButtonCtl(GamepadKeys.Button.DPAD_LEFT, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean dpadLeft) -> cs.schedule(new SingleMotorSetPower(0))));
    }
}
