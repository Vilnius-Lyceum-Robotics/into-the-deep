package org.firstinspires.ftc.teamcode.subsystems.arm;
import com.acmerobotics.dashboard.config.Config;

@Config
public interface ArmConfiguration {

    String MOTOR_NAME = "motor";
    String ENCODER_NAME = "Encoder";  //TODO FIX NAMING

    double acceleration = 6000;
    double deceleration = 2100;
    double maxVelocity = 400;

    double p = 0.03;
    double i = 0;
    double d = 0.0028;
    double v = 0.003;
    double a = 0.00016;

    double f_retracted = 0.08;
    double f_extended = 0.12;

    double minAngle = 0;
    double maxAngle = 150;

    double encoderTicksPerRotation = 8192;


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
