package org.firstinspires.ftc.teamcode.helpers.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class setClawState extends InstantCommand {

    public setClawState(ClawSubsystem claw, ClawConfiguration.TargetState state){
        super(()-> claw.setTargetState(state));
    }
}
