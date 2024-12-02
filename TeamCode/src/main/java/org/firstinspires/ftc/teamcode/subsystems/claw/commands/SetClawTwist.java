package org.firstinspires.ftc.teamcode.subsystems.claw.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class SetClawTwist extends InstantCommand {

    public SetClawTwist(ClawConfiguration.TargetTwist twist){
        super(()-> VLRSubsystem.getInstance(ClawSubsystem.class).setTargetTwist(twist));
    }
}
