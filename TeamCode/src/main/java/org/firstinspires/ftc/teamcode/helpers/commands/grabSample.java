package org.firstinspires.ftc.teamcode.helpers.commands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.commands.wrappers.setClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class grabSample extends SequentialCommandGroup {

    public grabSample(ClawSubsystem claw){
        addCommands(
                new setClawState(claw, ClawConfiguration.TargetState.CLOSED_FORCED),
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                            new WaitUntilCommand(claw::isSamplePresent),
                            new setClawState(claw, ClawConfiguration.TargetState.CLOSED_NORMAL)
                        ),
                        new SequentialCommandGroup(
                            new WaitCommand(500),
                            new setClawState(claw, ClawConfiguration.TargetState.OPEN)
                        )
                ));
        addRequirements(claw);
    }
}
