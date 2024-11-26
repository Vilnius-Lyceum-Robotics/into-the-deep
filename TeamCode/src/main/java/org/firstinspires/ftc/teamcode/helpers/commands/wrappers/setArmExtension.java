package org.firstinspires.ftc.teamcode.helpers.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration;

public class setArmExtension extends InstantCommand {

    public setArmExtension(ArmSubsystem arm, SlideConfiguration.TargetPosition extension){
        super(()-> arm.setTargetPosition(extension));
    }
}
