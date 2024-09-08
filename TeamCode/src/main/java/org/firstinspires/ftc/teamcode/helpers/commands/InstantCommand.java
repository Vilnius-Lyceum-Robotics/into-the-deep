package org.firstinspires.ftc.teamcode.helpers.commands;

import com.arcrobotics.ftclib.command.CommandBase;

public abstract class InstantCommand extends CommandBase {
    public abstract void run();

    @Override
    public void initialize() {
        run();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
