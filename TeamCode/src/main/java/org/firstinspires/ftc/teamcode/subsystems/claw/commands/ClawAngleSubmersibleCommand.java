package org.firstinspires.ftc.teamcode.subsystems.claw.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class ClawAngleSubmersibleCommand extends SequentialCommandGroup {
    public ClawAngleSubmersibleCommand() {
        ClawSubsystem claw = VLRSubsystem.getInstance(ClawSubsystem.class);
        ArmSubsystem arm = VLRSubsystem.getInstance(ArmSubsystem.class);

        if (arm.getState() == ArmSubsystem.ArmState.PRE_INTAKE) {
            if (claw.getTargetAngle() == ClawConfiguration.TargetAngle.DOWN) {
                claw.setState(ClawSubsystem.ClawState.CLOSED);
                addCommands(new SetClawAngle(ClawConfiguration.TargetAngle.UP),
                        new SetClawState(ClawConfiguration.TargetState.CLOSED_NORMAL));
            } else if (claw.getTargetAngle() == ClawConfiguration.TargetAngle.UP) {
                claw.setState(ClawSubsystem.ClawState.OPEN);
                addCommands(new SetClawAngle(ClawConfiguration.TargetAngle.DOWN),
                        new SetClawState(ClawConfiguration.TargetState.OPEN));
            }
        }
    }
}
