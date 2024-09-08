package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;

/**
 * Abstraction for secondary driver controls. All controls will be defined here.
 * For this to work well, all subsystems will be defined as singletons.
 */
public class SecondaryDriverControls extends DriverControls {
    public SecondaryDriverControls(Gamepad gamepad) {
        super(new GamepadEx(gamepad));

        // Define controls here
    }
}
