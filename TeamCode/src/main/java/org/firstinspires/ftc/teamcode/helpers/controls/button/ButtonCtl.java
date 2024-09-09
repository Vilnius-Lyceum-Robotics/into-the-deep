package org.firstinspires.ftc.teamcode.helpers.controls.button;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.helpers.controls.ControlDefinition;

import java.util.function.Consumer;

public class ButtonCtl implements ControlDefinition {
    GamepadKeys.Button button;
    Trigger trigger;
    boolean returnOnlyOnTrue;
    Consumer<Boolean> action;

    ButtonReader reader;

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
        if (reader == null) {
            GamepadButton btn = gamepad.getGamepadButton(button);
            reader = new ButtonReader(btn::get);
        }
        reader.readValue();
        switch (trigger) {
            case SIMPLE:
                result = reader.isDown();
                break;
            case WAS_JUST_RELEASED:
                result = reader.wasJustReleased();
                break;
            case WAS_JUST_PRESSED:
                result = reader.wasJustPressed();
                break;
            case STATE_JUST_CHANGED:
                result = reader.stateJustChanged();
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + trigger);
        }

        if (!returnOnlyOnTrue || result) action.accept(result);
    }
}
