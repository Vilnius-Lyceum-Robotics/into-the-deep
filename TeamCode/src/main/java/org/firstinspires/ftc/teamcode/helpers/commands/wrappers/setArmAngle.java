package org.firstinspires.ftc.teamcode.helpers.commands.wrappers;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

public class setArmAngle extends InstantCommand {

    public setArmAngle(ArmSubsystem arm, ArmConfiguration.TargetAngle angle) {
        super(()-> arm.setTargetAngle(angle));
    }

    public setArmAngle(ArmSubsystem arm, double extension) {
        super(()-> arm.setTargetPosition(extension));
    }
}
