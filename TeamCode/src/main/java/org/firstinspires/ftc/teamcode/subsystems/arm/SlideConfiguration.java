package org.firstinspires.ftc.teamcode.subsystems.arm;
import com.acmerobotics.dashboard.config.Config;

@Config
public class SlideConfiguration {
    public static String MOTOR_NAME_0 = "MotorArm1";
    public static String MOTOR_NAME_1 = "MotorArm2";
    public static String MOTOR_NAME_2 = "MotorArm3";
    public static String ENCODER_NAME = "MotorRightBack";

    public static double ACCELERATION = 35000;
    public static double DECELERATION_FAST = 20000;
    public static double DECELERATION_SLOW = 20000;
    public static double MAX_VELOCITY = 4000;
    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.018;
    public static double FEEDBACK_INTEGRAL_GAIN = 0;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.00075;
    public static double FEED_FORWARD_GAIN = 0;
    public static double VELOCITY_GAIN = 0.00037;
    public static double ACCELERATION_GAIN = 0.00003;

    public static double MIN_POSITION = 0;
    public static double MAX_POSITION = 1050;


    public enum TargetPosition {
        RETRACTED(0.05),
        INTAKE(0.5),
        DEPOSIT(0.8);

        public final double extension;
        TargetPosition(double extension) {
            this.extension = extension;
        }
    }
}
