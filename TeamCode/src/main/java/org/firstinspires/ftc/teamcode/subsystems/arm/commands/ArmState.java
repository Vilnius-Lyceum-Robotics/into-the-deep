package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

public class ArmState {
    private static STATE currentState = STATE.IN_ROBOT;

    public static STATE get() {return currentState;}

    public static class set extends InstantCommand{
        public set(STATE state){
            super(() -> currentState = state);
        }
    }

    public enum STATE {
        IN_ROBOT,
        INTAKE,
        DEPOSIT
    }
}
