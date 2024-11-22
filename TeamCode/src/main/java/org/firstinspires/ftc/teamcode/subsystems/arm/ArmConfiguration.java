package org.firstinspires.ftc.teamcode.subsystems.arm;
import com.acmerobotics.dashboard.config.Config;

@Config
public interface ArmConfiguration {

    String MOTOR_NAME = "motor";
    String ENCODER_NAME = "Encoder";  //TODO FIX NAMING

    double ACCELERATION = 6000;
    double DECELERATION = 2100;
    double MAX_VELOCITY = 400;

    double p = 0.03;
    double i = 0;
    double d = 0.0028;
    double VELOCITY_GAIN = 0.003;
    double ACCELERATION_GAIN = 0.00016;

    double RETRACTED_FEED_FORWARD_GAIN = 0.08;
    double EXTENDED_FEED_FORWARD_GAIN = 0.12;

    double MIN_ANGLE = 0;
    double MAX_ANGLE = 150;

    double ENCODER_TICKS_PER_ROTATION = 8192;


    enum TargetAngle {
        DOWN (0),
        INTAKE (0),
        DEPOSIT (125);

        public final double angleDegrees;
        TargetAngle(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }
    }
}
