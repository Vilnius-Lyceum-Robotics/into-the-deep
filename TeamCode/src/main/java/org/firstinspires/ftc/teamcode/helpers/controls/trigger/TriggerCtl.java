package org.firstinspires.ftc.teamcode.helpers.controls.trigger;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.helpers.controls.ControlDefinition;

import java.util.function.Consumer;

public class TriggerCtl implements ControlDefinition {
    GamepadKeys.Trigger trigger;
    Consumer<Double> action;

    public TriggerCtl(GamepadKeys.Trigger trigger, Consumer<Double> action) {
        this.trigger = trigger;
        this.action = action;
    }

    @Override
    public void run(GamepadEx gamepad) {
        action.accept(gamepad.getTrigger(trigger));
    }
}
