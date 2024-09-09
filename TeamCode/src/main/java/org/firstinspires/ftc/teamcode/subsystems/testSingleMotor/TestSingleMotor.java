package org.firstinspires.ftc.teamcode.subsystems.testSingleMotor;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class TestSingleMotor extends VLRSubsystem<TestSingleMotor> {

    Motor motor;

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        motor = new Motor(hardwareMap, "testMotor");
    }

    public void setPower(double power) {
        motor.set(power);
    }
}
