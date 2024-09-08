package org.firstinspires.ftc.teamcode.subsystems.lift.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftConfiguration;

public class LiftToggleCommand extends CommandBase {
    private final Lift lift;

    public LiftToggleCommand() {
        lift = VLRSubsystem.getInstance(Lift.class);
    }

    public void initialize() {
        if (lift.getPosition() > LiftConfiguration.LIFT_DOWN_POS + 10) {
            lift.runToBottom();
        } else {
            lift.runToTop();
        }
    }

    public boolean isFinished() {
        return Math.abs(lift.getTargetPosition() - lift.getPosition()) < LiftConfiguration.LIFT_POS_TOLERANCE;
    }
}
