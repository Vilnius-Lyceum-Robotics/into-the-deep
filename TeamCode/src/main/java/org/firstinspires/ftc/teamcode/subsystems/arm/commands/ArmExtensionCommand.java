package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem.ArmState.IN_ROBOT;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem.ArmState.PRE_INTAKE;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetArmAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmRotatingPartConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class ArmExtensionCommand extends SequentialCommandGroup {

    public ArmExtensionCommand() {
        ArmSubsystem arm = VLRSubsystem.getInstance(ArmSubsystem.class);
        if (arm.getState() == IN_ROBOT) {
            arm.setState(PRE_INTAKE);
            addCommands(
                    new SetClawTwist(ClawConfiguration.TargetTwist.NORMAL),
                    new SetArmAngle(ArmRotatingPartConfiguration.TargetAngle.INTAKE),
                    new SetArmExtension(SlideConfiguration.TargetPosition.INTAKE)
                    //new SetClawAngle(ClawConfiguration.TargetAngle.DOWN)
            );

        } else if (arm.getState() == PRE_INTAKE) {
            arm.setState(IN_ROBOT);
            addCommands(
                    new SetClawTwist(ClawConfiguration.TargetTwist.NORMAL),
                    new SetClawAngle(ClawConfiguration.TargetAngle.UP),
                    new SetClawState(ClawConfiguration.TargetState.CLOSED_NORMAL),
                    new SetArmExtension(SlideConfiguration.TargetPosition.RETRACTED),
                    new SetArmAngle(ArmRotatingPartConfiguration.TargetAngle.DOWN)
            );

        }
    }
}
