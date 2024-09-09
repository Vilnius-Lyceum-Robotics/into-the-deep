package org.firstinspires.ftc.teamcode.subsystems.testSingleMotor.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.testSingleMotor.TestSingleMotor;

public class SingleMotorSetPower extends CommandBase {
    private final double power;
    private final TestSingleMotor motor;

    public SingleMotorSetPower(double power) {
        this.power = power;
        motor = VLRSubsystem.getInstance(TestSingleMotor.class);
    }

    @Override
    public void initialize() {
        motor.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
