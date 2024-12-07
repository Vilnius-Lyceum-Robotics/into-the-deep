package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;

public class SetArmExtension extends InstantCommand {

    public SetArmExtension(ArmSlideConfiguration.TargetPosition extension) {
        super(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).setTargetPosition(extension));
    }
}
