package org.firstinspires.ftc.teamcode.subsystems.arm;
import com.acmerobotics.dashboard.config.Config;

@Config
public interface SlideConfiguration {
    String MOTOR_NAME_0 = "motor0";
    String MOTOR_NAME_1 = "motor1";
    String MOTOR_NAME_2 = "motor2";
    String ENCODER_NAME = "encoder"; // TODO FIX NAMING

    double ACCELERATION = 4500;
    double DECELERATION = 2000;
    double MAX_VELOCITY = 400;
    double FEEDBACK_PROPORTIONAL_GAIN = 0.025;
    double FEEDBACK_INTEGRAL_GAIN = 0;
    double FEEDBACK_DERIVATIVE_GAIN = 0.002;
    double FEED_FORWARD_GAIN = 0;
    double VELOCITY_GAIN = 0.0012;
    double ACCELERATION_GAIN = 0.00012;

    double MIN_POSITION = 0;
    double MAX_POSITION = 270;

    enum TargetPosition {
        RETRACTED(0.05),
        INTAKE(0.3),
        DEPOSIT(0.9);

        public final double extension;
        TargetPosition(double extension) {
            this.extension = extension;
        }
    }
}
