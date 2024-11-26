package org.firstinspires.ftc.teamcode.helpers.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class setClawTwist extends InstantCommand {

    public setClawTwist(ClawSubsystem claw, ClawConfiguration.TargetTwist twist){
        super(()-> claw.setTargetTwist(twist));
    }
}
