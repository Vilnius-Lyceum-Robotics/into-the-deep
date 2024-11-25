package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConfiguration {

    public static String MOTOR_NAME = "motor";
    public static String ENCODER_NAME = "Encoder";  //TODO FIX NAMING

    public static double ACCELERATION = 6000;
    public static double DECELERATION = 2100;
    public static double MAX_VELOCITY = 400;

    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.03;
    public static double FEEDBACK_INTEGRAL_GAIN = 0;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.0028;
    public static double VELOCITY_GAIN = 0.003;
    public static double ACCELERATION_GAIN = 0.00016;

    public static double RETRACTED_FEEDFORWARD_GAIN = 0.08;
    public static double EXTENDED_FEEDFORWARD_GAIN = 0.12;

    public static double MIN_ANGLE = 0;
    public static double MAX_ANGLE = 150;

    public static double ENCODER_TICKS_PER_ROTATION = 8192;


    public enum TargetAngle {
        DOWN(0),
        INTAKE(0),
        DEPOSIT(125);

        public double angleDegrees;

        TargetAngle(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }
    }
}
