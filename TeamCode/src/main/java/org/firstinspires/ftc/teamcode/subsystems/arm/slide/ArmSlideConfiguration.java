package org.firstinspires.ftc.teamcode.subsystems.arm.slide;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface ArmSlideConfiguration {
    String MOTOR_NAME_0 = "MotorArm1";
    String MOTOR_NAME_1 = "MotorArm2";
    String MOTOR_NAME_2 = "MotorArm3";
    String ENCODER_NAME = "MotorRightFront";

    double ACCELERATION = 22000;
    double DECELERATION_FAST = 10000;
    double MAX_VELOCITY = 1700;
    double FEEDBACK_PROPORTIONAL_GAIN = 0.02;
    double FEEDBACK_INTEGRAL_GAIN = 0;
    double FEEDBACK_DERIVATIVE_GAIN = 0.0004;
    double FEED_FORWARD_GAIN = 0.05;
    double VELOCITY_GAIN = 0.00025;
    double ACCELERATION_GAIN = 0.00002;

    double CREEP = 50;

    double ERROR_MARGIN = 8;

    double MIN_POSITION = 0;
    double HORIZONTAL_EXTENSION_LIMIT = 650;
    double MAX_POSITION = 1180;


    enum TargetPosition {
        RETRACTED(0.0035),
        INTAKE(0.45),
        DEPOSIT(0.95);

        public final double extension;

        TargetPosition(double extension) {
            this.extension = extension;
        }
    }
}
