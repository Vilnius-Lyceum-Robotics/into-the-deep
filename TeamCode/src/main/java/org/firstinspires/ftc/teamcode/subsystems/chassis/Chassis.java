package org.firstinspires.ftc.teamcode.subsystems.chassis;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class Chassis extends VLRSubsystem<Chassis> implements ChassisConfiguration {
    MotorEx MotorLeftFront;
    MotorEx MotorRightFront;
    MotorEx MotorLeftBack;
    MotorEx MotorRightBack;

    double motorPower = 0.0;

    MecanumDriveKinematics motorKinematics = new MecanumDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    );

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        MotorLeftFront = new MotorEx(hardwareMap, MOTOR_LEFT_FRONT);
        MotorRightFront = new MotorEx(hardwareMap, MOTOR_RIGHT_FRONT);
        MotorLeftBack = new MotorEx(hardwareMap, MOTOR_LEFT_BACK);
        MotorRightBack = new MotorEx(hardwareMap, MOTOR_RIGHT_BACK);

        MotorLeftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        MotorRightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        MotorLeftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        MotorRightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(Pose2d positionVector){
        ChassisSpeeds speeds = new ChassisSpeeds(positionVector.getX(), positionVector.getY(), positionVector.getHeading());
        MecanumDriveWheelSpeeds wheelSpeeds = motorKinematics.toWheelSpeeds(speeds);
        wheelSpeeds.normalize(1.0);

        MotorLeftFront.set(wheelSpeeds.frontLeftMetersPerSecond * motorPower);
        MotorRightFront.set(wheelSpeeds.frontRightMetersPerSecond * motorPower);
        MotorLeftBack.set(wheelSpeeds.rearLeftMetersPerSecond * motorPower);
        MotorRightBack.set(wheelSpeeds.rearRightMetersPerSecond * motorPower);
    }

    public void stop(){
        MotorLeftFront.stopMotor();
        MotorRightFront.stopMotor();
        MotorLeftBack.stopMotor();
        MotorRightBack.stopMotor();

        motorPower = 0.0;
    }

    public void setPower(double power){
        motorPower = Math.min(power, 1.0);
    }
}
