package org.firstinspires.ftc.teamcode.subsystems.lift.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftConfiguration;

public class LiftRunToPositionCommand extends CommandBase {
    private final Lift lift;
    private final int position;

    public LiftRunToPositionCommand(int position) {
        lift = VLRSubsystem.getInstance(Lift.class);
        this.position = position;
    }

    public void initialize() {
        lift.runToPosition(position);
    }

    public boolean isFinished() {
        return Math.abs(lift.getTargetPosition() - lift.getPosition()) < LiftConfiguration.LIFT_POS_TOLERANCE;
    }
}
