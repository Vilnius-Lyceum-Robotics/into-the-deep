package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmRotatingPartConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class SetArmAngle extends InstantCommand {

    public SetArmAngle(ArmRotatingPartConfiguration.TargetAngle angle) {
        super(()-> VLRSubsystem.getInstance(ArmSubsystem.class).setTargetAngle(angle));
    }

    public SetArmAngle(double extension) {
        super(()-> VLRSubsystem.getInstance(ArmSubsystem.class).setTargetPosition(extension));
    }
}
