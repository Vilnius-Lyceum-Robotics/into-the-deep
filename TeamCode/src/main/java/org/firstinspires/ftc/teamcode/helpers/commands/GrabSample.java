package org.firstinspires.ftc.teamcode.helpers.commands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class GrabSample extends SequentialCommandGroup {

    public GrabSample(){
        ClawSubsystem claw = VLRSubsystem.getInstance(ClawSubsystem.class);
        addCommands(
                new SetClawState(claw, ClawConfiguration.TargetState.CLOSED_FORCED),
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                            new WaitUntilCommand(claw::isSamplePresent),
                            new SetClawState(claw, ClawConfiguration.TargetState.CLOSED_NORMAL)
                        ),
                        new SequentialCommandGroup(
                            new WaitCommand(500),
                            new SetClawState(claw, ClawConfiguration.TargetState.OPEN)
                        )
                ));
        addRequirements(claw);
    }
}
