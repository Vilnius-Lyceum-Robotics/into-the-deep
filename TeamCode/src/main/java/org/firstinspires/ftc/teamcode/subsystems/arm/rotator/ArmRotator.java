package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class ArmRotator extends VLRSubsystem<ArmRotator> implements ArmRotatorConfiguration {
    MotorEx motor;
    Motor.Encoder encoder;
    ArmRotatorMotionProfile mp;

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, MOTOR_NAME);
        encoder = new MotorEx(hardwareMap, ENCODER_MOTOR_NAME).encoder;
        mp = new ArmRotatorMotionProfile();

        encoder.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);
    }

    public void setTargetPos(TargetPosition pos) {
        mp.setTargetPosition(pos.angle);
    }

    @Override
    public void periodic() {
        mp.update(motor.encoder.getDistance());
        motor.set(mp.getPower());
    }
}
