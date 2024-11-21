package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;


import com.acmerobotics.dashboard.config.Config;

@Config
public interface ArmRotatorConfiguration {

    String MOTOR_NAME = "armRotator";
    String ENCODER_MOTOR_NAME = "MotorRightBack"; // TODO

    double ENCODER_DISTANCE_PER_PULSE = 360 / 8192.0;

    enum TargetPosition {
        UP(0),
        DOWN(0);

        public final double angle;

        TargetPosition(double angle) {
            this.angle = angle;
        }
    }
}
