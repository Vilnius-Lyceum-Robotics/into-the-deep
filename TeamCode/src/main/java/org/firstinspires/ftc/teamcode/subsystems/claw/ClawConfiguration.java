package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConfiguration {
    public static String ANGLE_SERVO = "angle";
    public static String TWIST_SERVO = "twist";
    public static String GRAB_SERVO = "claw";
    public static String ANALOG_ENCODER_LEFT = "analog0";
    public static String ANALOG_ENCODER_RIGHT = "analog1";

    public static double angle_down_pos = 1;
    public static double angle_up_pos = 0;
    public static double angle_deposit_pos = 0.2;

    public static double twist_normal_pos = 0.04;
    public static double twist_flipped_pos = 0;

    public static double TWIST_MIN = 0.04;
    public static double TWIST_MAX = 0.96;

    public static double state_closed_normal_pos = 0;
    public static double state_closed_forced_pos = 0;
    public static double state_open_pos = 0.6;

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