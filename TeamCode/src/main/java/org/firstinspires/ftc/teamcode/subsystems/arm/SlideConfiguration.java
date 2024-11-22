package org.firstinspires.ftc.teamcode.subsystems.arm;
import com.acmerobotics.dashboard.config.Config;

@Config
public interface SlideConfiguration {
    String MOTOR_NAME_0 = "motor0";
    String MOTOR_NAME_1 = "motor1";
    String MOTOR_NAME_2 = "motor2";
    String ENCODER_NAME = "encoder"; // TODO FIX NAMING

    double acceleration = 4500;
    double deceleration = 2000;
    double maxVelocity = 400;
    double p = 0.025;
    double i = 0;
    double d = 0.002;
    double f = 0;
    double v = 0.0012;
    double a = 0.00012;

    double minPosition = 0;
    double maxPosition = 270;

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
