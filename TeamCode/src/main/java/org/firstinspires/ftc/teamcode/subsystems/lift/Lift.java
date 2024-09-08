package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class Lift extends VLRSubsystem<Lift> implements LiftConfiguration {
    Motor liftMotor;

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        liftMotor = new Motor(hardwareMap, LIFT_MOTOR);
    }

    public void runToTop() {
        liftMotor.set(LIFT_UP_POS);
    }

    public void runToBottom() {
        liftMotor.set(LIFT_DOWN_POS);
    }

    public void runToPosition(int position) {
        liftMotor.set(position);
    }

    public double getPosition() {
        return liftMotor.getCurrentPosition();
    }

    public double getTargetPosition() {
        return liftMotor.motor.getTargetPosition();
    }
}
