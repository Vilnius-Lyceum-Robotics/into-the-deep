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

/**
 * Abstraction for primary driver controls. All controls will be defined here.
 * For this to work well, all subsystems will be defined as singletons.
 */
public class PrimaryDriverControls extends DriverControls {
    public PrimaryDriverControls(Gamepad gamepad) {
        super(new GamepadEx(gamepad));

        CommandScheduler cs = CommandScheduler.getInstance();

        add(new ButtonCtl(GamepadKeys.Button.LEFT_BUMPER, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean lb) -> cs.schedule(new ClawToggleCommand())));
        add(new ButtonCtl(GamepadKeys.Button.RIGHT_BUMPER, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean rb) -> cs.schedule(new LiftToggleCommand())));
        add(new ButtonCtl(GamepadKeys.Button.A, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean a) -> cs.schedule(new LiftRunToPositionCommand(5000))));
    }
}
