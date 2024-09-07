package org.firstinspires.ftc.teamcode.controls.def.trigger;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.controls.def.ControlDefinition;

import java.util.function.Consumer;

public class TriggerCtlDef implements ControlDefinition {
    GamepadKeys.Trigger trigger;
    Consumer<Double> action;

    public TriggerCtlDef(GamepadKeys.Trigger trigger, Consumer<Double> action) {
        this.trigger = trigger;
        this.action = action;
    }

    @Override
    public void run(GamepadEx gamepad) {
        action.accept(gamepad.getTrigger(trigger));
    }
}
