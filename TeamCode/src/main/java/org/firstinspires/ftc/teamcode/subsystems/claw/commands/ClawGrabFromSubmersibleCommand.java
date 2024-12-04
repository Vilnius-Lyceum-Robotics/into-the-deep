package org.firstinspires.ftc.teamcode.subsystems.claw.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.helpers.commands.GrabSample;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmRotatingPartConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetArmAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class ClawGrabFromSubmersibleCommand extends SequentialCommandGroup {
    public ClawGrabFromSubmersibleCommand() {
        ClawSubsystem claw = VLRSubsystem.getInstance(ClawSubsystem.class);
        ArmSubsystem arm = VLRSubsystem.getInstance(ArmSubsystem.class);

        if (arm.getState() == ArmSubsystem.ArmState.PRE_INTAKE) {
            if (claw.getState() == ClawSubsystem.ClawState.OPEN) {
                claw.setState(ClawSubsystem.ClawState.CLOSED);
                addCommands(
                        new SetArmAngle(ArmRotatingPartConfiguration.TargetAngle.DOWN),
                        new GrabSample(),
                        new SetArmAngle(ArmRotatingPartConfiguration.TargetAngle.INTAKE)
                );
            } else {
                claw.setState(ClawSubsystem.ClawState.OPEN);
                addCommands(
                        new SetArmAngle(ArmRotatingPartConfiguration.TargetAngle.DOWN),
                        new SetClawState(ClawConfiguration.TargetState.OPEN),
                        new SetArmAngle(ArmRotatingPartConfiguration.TargetAngle.INTAKE)
                );
            }
        } else if (arm.getState() == ArmSubsystem.ArmState.FIRST_BASKET) {
            if (claw.getState() == ClawSubsystem.ClawState.OPEN) {
                claw.setState(ClawSubsystem.ClawState.CLOSED);
                addCommands(
                        new SetClawState(ClawConfiguration.TargetState.CLOSED_NORMAL)
                );
            } else {
                claw.setState(ClawSubsystem.ClawState.OPEN);
                addCommands(
                        new SetClawState(ClawConfiguration.TargetState.OPEN)
                );
            }
        }
    }
}
