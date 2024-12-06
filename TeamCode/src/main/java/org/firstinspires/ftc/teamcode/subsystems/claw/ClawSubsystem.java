package org.firstinspires.ftc.teamcode.subsystems.claw;

import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.ANALOG_ENCODER_LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.ANALOG_ENCODER_RIGHT;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.ANGLE_SERVO;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.GRAB_SERVO;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TWIST_MAX;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TWIST_MIN;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TWIST_SERVO;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.analog_voltage_left;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.analog_voltage_right;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.angle_deposit_pos;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.angle_down_pos;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.angle_up_pos;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.state_closed_forced_pos;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.state_closed_normal_pos;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.state_open_pos;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.twist_flipped_pos;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.twist_normal_pos;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;


public class ClawSubsystem extends VLRSubsystem<ClawSubsystem> {
    private Servo angleServo, twistServo, grabServos;
    private AnalogInput analogLeft, analogRight;

    public ClawState state = ClawState.OPEN;

    private double twistIncrement;

    private ClawConfiguration.TargetAngle targetAngle = ClawConfiguration.TargetAngle.UP;

    public enum ClawState {
        OPEN, CLOSED
    }

    public ClawState getState() {
        return state;
    }

    public void setState(ClawState state) {
        this.state = state;
    }


    protected void initialize(HardwareMap hardwareMap) {
        angleServo = hardwareMap.get(Servo.class, ANGLE_SERVO);
        twistServo = hardwareMap.get(Servo.class, TWIST_SERVO);
        grabServos = hardwareMap.get(Servo.class, GRAB_SERVO);

        analogLeft = hardwareMap.get(AnalogInput.class, ANALOG_ENCODER_LEFT);
        analogRight = hardwareMap.get(AnalogInput.class, ANALOG_ENCODER_RIGHT);

        setTargetAngle(ClawConfiguration.TargetAngle.UP);
        setTargetTwist(ClawConfiguration.TargetTwist.NORMAL);
        setTargetState(ClawConfiguration.TargetState.CLOSED_NORMAL);
    }


    public void setTargetAngle(ClawConfiguration.TargetAngle targetAngle) {
        this.targetAngle = targetAngle;
        switch (targetAngle) {
            case UP:
                angleServo.setPosition(angle_up_pos);
                break;
            case DOWN:
                angleServo.setPosition(angle_down_pos);
                break;
            case DEPOSIT:
                angleServo.setPosition(angle_deposit_pos);
                break;
        }
    }

    public ClawConfiguration.TargetAngle getTargetAngle() {
        return targetAngle;
    }


    public void setTargetTwist(ClawConfiguration.TargetTwist twistAngle) {
        switch (twistAngle) {
            case NORMAL:
                twistServo.setPosition(twist_normal_pos);
                break;
            case FLIPPED:
                twistServo.setPosition(twist_flipped_pos);
                break;
        }
    }

    public void incrementTwist(double increment) {
        twistIncrement = increment;
    }


    public void setTargetState(ClawConfiguration.TargetState targetState) {
        switch (targetState) {
            case OPEN:
                grabServos.setPosition(state_open_pos);
                break;
            case CLOSED_NORMAL:
                grabServos.setPosition(state_closed_normal_pos);
                break;
            case CLOSED_FORCED:
                grabServos.setPosition(state_closed_forced_pos);
                break;
        }
        twistIncrement = 0;
    }


    public boolean isSamplePresent() {
        return analogLeft.getVoltage() > analog_voltage_left && analogRight.getVoltage() > analog_voltage_right;
    }

    @Override
    public void periodic() {
        twistServo.setPosition(Math.max(Math.min(twistServo.getPosition() + twistIncrement, TWIST_MAX), TWIST_MIN));
    }
}