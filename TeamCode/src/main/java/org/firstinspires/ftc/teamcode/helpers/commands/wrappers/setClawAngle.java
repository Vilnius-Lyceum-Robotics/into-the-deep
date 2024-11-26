package org.firstinspires.ftc.teamcode.helpers.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class setClawAngle extends InstantCommand {

    public setClawAngle(ClawSubsystem claw, ClawConfiguration.TargetAngle angle){
        super(()-> claw.setTargetAngle(angle));
    }
}
