package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmRotatingPartConfiguration {

    public static String MOTOR_NAME = "MotorRotator";
    public static String ENCODER_NAME = "MotorLeftFront";

    public static double ACCELERATION = 6000;
    public static double DECELERATION = 4000;
    public static double MAX_VELOCITY = 480;

    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.15;
    public static double FEEDBACK_INTEGRAL_GAIN = 0;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.007;
    public static double VELOCITY_GAIN = 0.009;
    public static double ACCELERATION_GAIN = 0.0003;

    public static double RETRACTED_FEEDFORWARD_GAIN = 0.15;
    public static double EXTENDED_FEEDFORWARD_GAIN = 0.4;

    public static double MIN_ANGLE = 0;
    public static double MAX_ANGLE = 150;

    public static double ENCODER_TICKS_PER_ROTATION = 8192;


    public enum TargetAngle {
        DOWN(0),
        INTAKE(0),
        DEPOSIT(90);

        public final double angleDegrees;

        TargetAngle(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }
    }
}
