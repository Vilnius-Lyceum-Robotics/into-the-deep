package org.firstinspires.ftc.teamcode.subsystems.arm.slide;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmSlideConfiguration {
    public static String MOTOR_NAME_0 = "MotorArm1";
    public static String MOTOR_NAME_1 = "MotorArm2";
    public static String MOTOR_NAME_2 = "MotorArm3";
    public static String ENCODER_NAME = "MotorRightFront";

    public static double ACCELERATION = 22000;
    public static double DECELERATION_FAST = 10000;
    public static double MAX_VELOCITY = 1700;
    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.02;
    public static double FEEDBACK_INTEGRAL_GAIN = 0.0001;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.0004;
    public static double FEED_FORWARD_GAIN = 0.05;
    public static double VELOCITY_GAIN = 0.00025;
    public static double ACCELERATION_GAIN = 0.00002;

    public static double CREEP = 50;

    public static double ERROR_MARGIN = 10;

    public static double a = 0.6;

    public static double MIN_POSITION = 0;
    public static double HORIZONTAL_EXTENSION_LIMIT = 650;
    public static double MAX_POSITION = 1200;


    public enum TargetPosition {
        RETRACTED(0.0035),
        INTAKE(0.45),
        DEPOSIT(0.985);

        public final double extension;

        TargetPosition(double extension) {
            this.extension = extension;
        }
    }
}
