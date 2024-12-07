package org.firstinspires.ftc.teamcode.subsystems.chassis;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.helpers.MecanumDriveController;


public class Chassis extends VLRSubsystem<Chassis> implements ChassisConfiguration {
    MotorEx MotorLeftFront;
    MotorEx MotorRightFront;
    MotorEx MotorLeftBack;
    MotorEx MotorRightBack;

    double motorPower = 1.0;

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

        MotorLeftBack.setRunMode(Motor.RunMode.RawPower);
        MotorRightBack.setRunMode(Motor.RunMode.RawPower);
        MotorRightFront.setRunMode(Motor.RunMode.RawPower);
        MotorLeftFront.setRunMode(Motor.RunMode.RawPower);

        MotorRightBack.setInverted(true);
        MotorRightFront.setInverted(true);
    }

    public void drive(Pose2d positionVector) {
        this.driveMotors(new MecanumDriveController(
                positionVector.getX(),
                positionVector.getY(),
                positionVector.getHeading() * 0.1
        ));
    }

    public void drive(double xSpeed, double ySpeed, double zRotation) {
        this.driveMotors(new MecanumDriveController(xSpeed, ySpeed, zRotation));
    }

    private void driveMotors(MecanumDriveController driveController) {
        driveController.normalize(1.0);

        MotorLeftFront.set(driveController.frontLeftMetersPerSecond * motorPower);
        MotorRightFront.set(driveController.frontRightMetersPerSecond * motorPower);
        MotorLeftBack.set(driveController.rearLeftMetersPerSecond * motorPower);
        MotorRightBack.set(driveController.rearRightMetersPerSecond * motorPower);
    }

    public void stop() {
        MotorLeftFront.stopMotor();
        MotorRightFront.stopMotor();
        MotorLeftBack.stopMotor();
        MotorRightBack.stopMotor();

        motorPower = 0.0;
    }

    public void setPower(double power) {
        motorPower = Math.min(power, 1.0);
    }
}