package org.firstinspires.ftc.teamcode.subsystems.chassis;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

@Config
public interface ChassisConfiguration {
    String MOTOR_LEFT_FRONT = "MotorLeftFront";
    String MOTOR_RIGHT_FRONT = "MotorRightFront";
    String MOTOR_LEFT_BACK = "MotorLeftBack";
    String MOTOR_RIGHT_BACK = "MotorRightBack";

    /**
     * Locations are relative to the center of the robot
     */
    Translation2d frontLeftLocation = new Translation2d(0, 0);
    Translation2d frontRightLocation = new Translation2d(0, 0);
    Translation2d backLeftLocation = new Translation2d(0, 0);
    Translation2d backRightLocation = new Translation2d(0, 0);
}
