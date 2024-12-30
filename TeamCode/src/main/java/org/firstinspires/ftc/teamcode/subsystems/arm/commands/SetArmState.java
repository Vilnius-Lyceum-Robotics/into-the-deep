package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;

public class SetArmState extends InstantCommand {
    public SetArmState(ArmState.State stateValue) {
        super(() -> ArmState.set(stateValue));
    }
}
