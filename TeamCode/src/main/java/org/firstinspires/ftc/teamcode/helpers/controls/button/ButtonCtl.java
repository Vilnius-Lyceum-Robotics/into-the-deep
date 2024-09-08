package org.firstinspires.ftc.teamcode.helpers.controls.button;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.helpers.controls.ControlDefinition;

import java.util.function.Consumer;

public class ButtonCtl implements ControlDefinition {
    GamepadKeys.Button button;
    Trigger trigger;
    boolean returnOnlyOnTrue;
    Consumer<Boolean> action;

    public enum Trigger {
        SIMPLE,
        WAS_JUST_RELEASED,
        WAS_JUST_PRESSED,
        STATE_JUST_CHANGED
    }
    public ButtonCtl(GamepadKeys.Button button, Trigger trigger, boolean returnOnlyOnTrue, Consumer<Boolean> action) {
        this.button = button;
        this.trigger = trigger;
        this.returnOnlyOnTrue = returnOnlyOnTrue;
        this.action = action;
    }

    public ButtonCtl(GamepadKeys.Button button, Trigger trigger, Consumer<Boolean> action) {
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
