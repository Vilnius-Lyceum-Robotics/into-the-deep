package org.firstinspires.ftc.teamcode.subsystems.arm.slide;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface ArmSlideConfiguration {
    String MOTOR_NAME_0 = "MotorArm1";
    String MOTOR_NAME_1 = "MotorArm2";
    String MOTOR_NAME_2 = "MotorArm3";
    String ENCODER_NAME = "MotorRightFront";

    double ACCELERATION = 35000;
    double DECELERATION_FAST = 12000;
    double MAX_VELOCITY = 4000;
    double FEEDBACK_PROPORTIONAL_GAIN = 0.015;
    double FEEDBACK_INTEGRAL_GAIN = 0;
    double FEEDBACK_DERIVATIVE_GAIN = 0.0006;
    double FEED_FORWARD_GAIN = 0.075;
    double VELOCITY_GAIN = 0.00038;
    double ACCELERATION_GAIN = 0.000028;

    double MIN_POSITION = 0;
    double MAX_POSITION = 1180;


    enum TargetPosition {
        RETRACTED(0.05),
        INTAKE(0.5),
        DEPOSIT(1);

        public final double extension;

        TargetPosition(double extension) {
            this.extension = extension;
        }
    }
}
