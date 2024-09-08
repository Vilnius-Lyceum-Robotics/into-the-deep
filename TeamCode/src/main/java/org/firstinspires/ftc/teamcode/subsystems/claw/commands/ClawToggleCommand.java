package org.firstinspires.ftc.teamcode.subsystems.claw.commands;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;

public class ClawToggleCommand extends InstantCommand {
    final Claw claw;

    public ClawToggleCommand() {
        claw = VLRSubsystem.getInstance(Claw.class);
        addRequirements(claw);
    }

    public void run() {
        if (claw.isOpen()) claw.close();
        else claw.open();
    }
}
