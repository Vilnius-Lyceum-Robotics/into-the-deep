package org.firstinspires.ftc.teamcode.subsystems.claw;
import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConfiguration {
    public static String SERVO_NAME_0 = "Servo1";
    public static String SERVO_NAME_1 = "Servo2";
    public static String SERVO_NAME_2 = "Servo3";
    public static String ANALOG_ENCODER_NAME_0 = "Analog0";
    public static String ANALOG_ENCODER_NAME_1 = "Analog1";

    public static double angle_down_pos = 0;
    public static double angle_up_pos = 0;
    public static double angle_deposit_pos = 0;

    public static double twist_normal_pos = 0;
    public static double twist_flipped_pos = 0;

    public static double state_closed_normal_pos = 0;
    public static double state_closed_forced_pos = 0;
    public static double state_open_pos = 0;

    public static double analog_voltage_left = 0;
    public static double analog_voltage_right = 0;


    public enum TargetAngle {
        DOWN,
        UP,
        DEPOSIT
    }

    public enum TargetTwist {
        NORMAL,
        FLIPPED
    }

    public enum TargetState {
        CLOSED_NORMAL,
        CLOSED_FORCED,
        OPEN
    }
}