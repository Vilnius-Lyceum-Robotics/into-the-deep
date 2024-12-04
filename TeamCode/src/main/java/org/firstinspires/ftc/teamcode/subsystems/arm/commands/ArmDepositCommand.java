package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmRotatingPartConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

public class ArmDepositCommand extends SequentialCommandGroup {
    public ArmDepositCommand() {
        ArmSubsystem arm = VLRSubsystem.getInstance(ArmSubsystem.class);
        if (arm.getState() == ArmSubsystem.ArmState.IN_ROBOT) {
            arm.setState(ArmSubsystem.ArmState.FIRST_BASKET);
            addCommands(
                    new SetArmAngle(ArmRotatingPartConfiguration.TargetAngle.DEPOSIT),
                    new SetArmExtension(SlideConfiguration.TargetPosition.DEPOSIT),
                    new SetClawAngle(ClawConfiguration.TargetAngle.DEPOSIT)
            );
        } else if (arm.getState() == ArmSubsystem.ArmState.FIRST_BASKET) {
            arm.setState(ArmSubsystem.ArmState.IN_ROBOT);
            addCommands(
                    new SetClawAngle(ClawConfiguration.TargetAngle.UP),
                    new SetClawState(ClawConfiguration.TargetState.CLOSED_NORMAL),
                    new SetArmExtension(SlideConfiguration.TargetPosition.RETRACTED),
                    new WaitCommand(380),
                    new SetArmAngle(ArmRotatingPartConfiguration.TargetAngle.DOWN)
            );
        }
    }
}
