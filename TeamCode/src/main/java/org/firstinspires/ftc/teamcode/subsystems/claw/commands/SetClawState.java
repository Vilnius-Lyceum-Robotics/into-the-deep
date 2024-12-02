package org.firstinspires.ftc.teamcode.subsystems.claw.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class SetClawState extends InstantCommand {

    public SetClawState(ClawConfiguration.TargetState state){
        super(()-> VLRSubsystem.getInstance(ClawSubsystem.class).setTargetState(state));
    }
}
