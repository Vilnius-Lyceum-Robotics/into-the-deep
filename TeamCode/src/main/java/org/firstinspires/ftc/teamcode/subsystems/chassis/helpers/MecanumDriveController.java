package org.firstinspires.ftc.teamcode.subsystems.chassis.helpers;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

public class MecanumDriveController extends MecanumDriveWheelSpeeds implements MecanumDriveControllerConfiguration {
    public MecanumDriveController(Pose2d vectorPosition){
        /**
         * Calculations here:
         * https://gm0.org/en/latest/docs/software/concepts/kinematics.html#id3
         */
        super(
                vectorPosition.getX() - vectorPosition.getY() - ( 2 * BASE_TRACK_RADIUS * vectorPosition.getHeading()),
                vectorPosition.getX() + vectorPosition.getY() + ( 2 * BASE_TRACK_RADIUS * vectorPosition.getHeading()),
                vectorPosition.getX() + vectorPosition.getY() - ( 2 * BASE_TRACK_RADIUS * vectorPosition.getHeading()),
                vectorPosition.getX() - vectorPosition.getY() + ( 2 * BASE_TRACK_RADIUS * vectorPosition.getHeading())
        );
    }
}
