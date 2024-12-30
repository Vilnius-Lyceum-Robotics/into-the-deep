package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmRotatorConfiguration {

    public static String MOTOR_NAME = "MotorRotator";
    public static String ENCODER_NAME = "MotorLeftFront";

    public static double ACCELERATION = 7000;
    public static double DECELERATION = 1500;
    public static double MAX_VELOCITY = 300;

    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.059;
    public static double FEEDBACK_INTEGRAL_GAIN = 0;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.0029;
    public static double VELOCITY_GAIN = 0.0035;
    public static double ACCELERATION_GAIN = 0.0001;

    public static double RETRACTED_FEEDFORWARD_GAIN = 0.12;
    public static double EXTENDED_FEEDFORWARD_GAIN = 0.5;

    public static double ERROR_MARGIN = 2;

    public static double MIN_ANGLE = 0;
    public static double MAX_ANGLE = 150;

    public static double ENCODER_TICKS_PER_ROTATION = 8192;


    public enum TargetAngle {
        DOWN(0),
        INTAKE(0),
        DEPOSIT(110);

        public final double angleDegrees;

        TargetAngle(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }
    }

    public enum RotatorState {
        IN_ROBOT,
        PRE_INTAKE,
        INTAKE,
        SECOND_BASKET,
        FIRST_BASKET
    }
}
