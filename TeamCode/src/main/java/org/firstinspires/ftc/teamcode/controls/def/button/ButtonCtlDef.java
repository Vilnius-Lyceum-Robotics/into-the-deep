package org.firstinspires.ftc.teamcode.controls.def.button;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.controls.def.ControlDefinition;

import java.util.function.Consumer;

public class ButtonCtlDef implements ControlDefinition {
    GamepadKeys.Button button;
    ButtonCtlTrigger trigger;
    boolean returnOnlyOnTrue;
    Consumer<Boolean> action;

    public ButtonCtlDef(GamepadKeys.Button button, ButtonCtlTrigger trigger, boolean returnOnlyOnTrue, Consumer<Boolean> action) {
        this.button = button;
        this.trigger = trigger;
        this.returnOnlyOnTrue = returnOnlyOnTrue;
        this.action = action;
    }

    public ButtonCtlDef(GamepadKeys.Button button, ButtonCtlTrigger trigger, Consumer<Boolean> action) {
        this(button, trigger, false, action);
    }

    @Override
    public void run(GamepadEx gamepad) {
        boolean result;
        switch (trigger) {
            case SIMPLE:
                result = gamepad.getButton(button);
                break;
            case WAS_JUST_RELEASED:
                result = gamepad.wasJustReleased(button);
                break;
            case WAS_JUST_PRESSED:
                result = gamepad.wasJustPressed(button);
                break;
            case STATE_JUST_CHANGED:
                result = gamepad.stateJustChanged(button);
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + trigger);
        }

        if (!returnOnlyOnTrue || result) action.accept(result);
    }
}
