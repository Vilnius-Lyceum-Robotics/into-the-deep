package org.firstinspires.ftc.teamcode.helpers.controls;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import java.util.List;
import java.util.function.BiConsumer;

/**
 * Abstraction for gamepad.
 * Defines controls using ControlDefinitions instead of actually writing if statements or similar.
 */
public class DriverControls {
    GamepadEx gamepad;
    List<ControlDefinition> controls;

    BiConsumer<Double, Double> leftStickHandler;
    BiConsumer<Double, Double> rightStickHandler;

    public DriverControls(GamepadEx gamepad) {
        this.gamepad = gamepad;
    }

    public void add(ControlDefinition control) {
        this.controls.add(control);
    }

    public void addLeftStickHandler(BiConsumer<Double, Double> control) {
        leftStickHandler = control;
    }

    public void addRightStickHandler(BiConsumer<Double, Double> control) {
        rightStickHandler = control;
    }

    public void update() {
        if (leftStickHandler != null)
            leftStickHandler.accept(gamepad.getLeftY(), gamepad.getLeftX());
        if (rightStickHandler != null)
            rightStickHandler.accept(gamepad.getRightY(), gamepad.getRightX());

        for (ControlDefinition control : controls) {
            control.run(gamepad);
        }
    }
}
