package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration;

public class SetArmExtension extends InstantCommand {

    public SetArmExtension(SlideConfiguration.TargetPosition extension) {
        super(() -> VLRSubsystem.getInstance(ArmSubsystem.class).setTargetPosition(extension));
    }
}
