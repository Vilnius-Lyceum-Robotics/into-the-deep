package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.commands.TelemetryDebugCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetTwist;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class SetArmState_InRobot extends SequentialCommandGroup {

    public SetArmState_InRobot(Telemetry telemetry) {
        ArmRotatorSubsystem arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
        ArmSlideSubsystem slides = VLRSubsystem.getInstance(ArmSlideSubsystem.class);

        
        if (ArmState.get() == ArmState.STATE.INTAKE){
            addCommands(
                    new SetClawState(TargetState.CLOSED_NORMAL),
                    new WaitCommand(100),
                    new SetClawAngle(TargetAngle.UP),
                    new SetClawTwist(TargetTwist.NORMAL),
                    new WaitCommand(80),
                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                    new WaitUntilCommand(slides::reachedTargetPosition),
                    new ArmState.set(ArmState.STATE.IN_ROBOT)
            );
        }
        else if (ArmState.get() == ArmState.STATE.DEPOSIT){
            addCommands(
                    new ParallelCommandGroup(
                            new PerpetualCommand(new TelemetryDebugCommand(telemetry, "COMMAND IN SET ARMS STATE IN ROBOT")),
                            new SequentialCommandGroup(
                                    new SetClawState(TargetState.OPEN),
                                    new WaitCommand(200),
                                    new SetClawAngle(TargetAngle.DOWN),
                                    new WaitCommand(100),
                                    new SetClawState(TargetState.CLOSED_NORMAL),
                                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                    new WaitCommand(200),
                                    new SetClawAngle(TargetAngle.DEPOSIT),
                                    new WaitUntilCommand(slides::reachedTargetPosition),
                                    new SetClawAngle(TargetAngle.UP),
                                    new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.DOWN),
                                    new WaitUntilCommand(arm::reachedTargetPosition),

                                    new ArmState.set(ArmState.STATE.IN_ROBOT)
                            )
                    )
            );
        }
        else{
            addCommands(
                    new PerpetualCommand(new TelemetryDebugCommand(telemetry, "SOMETHING WITH ARM STATE IS NOT WORKING"))
            );
        }
        addRequirements(arm, slides);
    }

    public SetArmState_InRobot(){
        this(FtcDashboard.getInstance().getTelemetry());
    }
}
