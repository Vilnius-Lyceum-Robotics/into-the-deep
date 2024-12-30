package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;

public class SetArmState_Deposit extends SequentialCommandGroup {

    public SetArmState_Deposit() {
        ArmRotatorSubsystem arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
        ArmSlideSubsystem slides = VLRSubsystem.getInstance(ArmSlideSubsystem.class);

        if (ArmState.get() != ArmState.STATE.DEPOSIT) {
            addCommands(
                    new CustomConditionalCommand(
                            new SetArmState_InRobot(),
                            () -> ArmState.get() == ArmState.STATE.INTAKE
                    ),

                    new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.DEPOSIT),
                    new WaitUntilCommand(() -> arm.getAngleDegrees() >= 30),
                    new SetClawAngle(TargetAngle.DOWN),

                    new WaitUntilCommand(arm::reachedTargetPosition),
                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.DEPOSIT),

                    new WaitUntilCommand(slides::reachedTargetPosition),
                    new SetClawAngle(TargetAngle.DEPOSIT),
                    new ArmState.set(ArmState.STATE.DEPOSIT)
            );
            addRequirements(arm, slides);
        }
    }
}