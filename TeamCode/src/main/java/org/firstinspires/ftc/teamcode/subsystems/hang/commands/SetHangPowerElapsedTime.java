package org.firstinspires.ftc.teamcode.subsystems.hang.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;

public class SetHangPowerElapsedTime extends CommandBase {
    private ElapsedTime timer;
    private double time;
    private double power;
    private HangSubsystem hangSubsystem;

    public SetHangPowerElapsedTime(double power, double time){
        timer = new ElapsedTime();
        this.time = time;
        this.power = power;
        hangSubsystem = VLRSubsystem.getInstance(HangSubsystem.class);
    }

    @Override
    public void initialize(){
        hangSubsystem.setPower(power);
    }


    @Override
    public boolean isFinished(){
        return timer.seconds() > time;
    }

    @Override
    public void end(boolean interrupted){
        hangSubsystem.setPower(0);
    }
}
