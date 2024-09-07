package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.controls.def.DriverControls;
import org.firstinspires.ftc.teamcode.controls.def.button.ButtonCtlDef;
import org.firstinspires.ftc.teamcode.controls.def.button.ButtonCtlTrigger;

/**
 * Abstraction for primary driver controls. All controls will be defined here.
 * For this to work well, all subsystems will be defined as singletons.
 */
public class PrimaryDriverControls extends DriverControls {
    public PrimaryDriverControls(Gamepad gamepad) {
        super(new GamepadEx(gamepad));

        // Example control definition
        add(new ButtonCtlDef(GamepadKeys.Button.A, ButtonCtlTrigger.SIMPLE, (Boolean a) -> System.out.println("Btn A status: " + a)));
        add(new ButtonCtlDef(GamepadKeys.Button.A, ButtonCtlTrigger.SIMPLE, true, (Boolean a) -> System.out.println("Btn A pressed!")));

        add(new ButtonCtlDef(GamepadKeys.Button.B, ButtonCtlTrigger.WAS_JUST_PRESSED, true, (Boolean b) -> System.out.println("B just pressed!")));
        add(new ButtonCtlDef(GamepadKeys.Button.B, ButtonCtlTrigger.WAS_JUST_RELEASED, true, (Boolean b) -> System.out.println("B just released!")));

        add(new ButtonCtlDef(GamepadKeys.Button.X, ButtonCtlTrigger.STATE_JUST_CHANGED, true, (Boolean x) -> System.out.println("X state just changed!")));
    }
}
