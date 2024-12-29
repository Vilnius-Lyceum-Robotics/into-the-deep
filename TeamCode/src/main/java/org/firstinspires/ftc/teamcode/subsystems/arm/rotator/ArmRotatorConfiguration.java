package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface ArmRotatorConfiguration {

    String MOTOR_NAME = "MotorRotator";
    String ENCODER_NAME = "MotorLeftFront";

    double ACCELERATION = 7000;
    double DECELERATION = 1500;
    double MAX_VELOCITY = 300;

    double FEEDBACK_PROPORTIONAL_GAIN = 0.059;
    double FEEDBACK_INTEGRAL_GAIN = 0;
    double FEEDBACK_DERIVATIVE_GAIN = 0.0029;
    double VELOCITY_GAIN = 0.0035;
    double ACCELERATION_GAIN = 0.0001;

    double RETRACTED_FEEDFORWARD_GAIN = 0.12;
    double EXTENDED_FEEDFORWARD_GAIN = 0.5;

    double ERROR_MARGIN = 2;

    double MIN_ANGLE = 0;
    double MAX_ANGLE = 150;

    double ENCODER_TICKS_PER_ROTATION = 8192;


    enum TargetAngle {
        DOWN(0),
        INTAKE(0),
        DEPOSIT(110);

        public final double angleDegrees;

        TargetAngle(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }
    }

    enum RotatorState {
        IN_ROBOT,
        PRE_INTAKE,
        INTAKE,
        SECOND_BASKET,
        FIRST_BASKET
    }

    enum ArmState {
        IN_ROBOT,
        INTAKE,
        DEPOSIT
    }
}
