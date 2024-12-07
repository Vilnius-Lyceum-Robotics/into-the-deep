package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;

public class SetArmAngle extends InstantCommand {

    public SetArmAngle(ArmRotatorConfiguration.TargetAngle angle) {
        super(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).setTargetAngle(angle));
    }

    public SetArmAngle(double extension) {
        super(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).setTargetPosition(extension));
    }
}
