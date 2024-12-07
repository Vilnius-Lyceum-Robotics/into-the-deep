package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;


public class ClawSubsystem extends VLRSubsystem<ClawSubsystem> implements ClawConfiguration {
    private Servo angleServo, twistServo, grabServos;
    private AnalogInput analogLeft, analogRight;

    public ClawState clawState = ClawState.OPEN;

    private double twistIncrement;

    private TargetAngle targetAngle = TargetAngle.UP;


    public ClawState getClawState() {
        return clawState;
    }

    public void setClawState(ClawState clawState) {
        this.clawState = clawState;
    }


    protected void initialize(HardwareMap hardwareMap) {
        angleServo = hardwareMap.get(Servo.class, ANGLE_SERVO);
        twistServo = hardwareMap.get(Servo.class, TWIST_SERVO);
        grabServos = hardwareMap.get(Servo.class, GRAB_SERVO);

        analogLeft = hardwareMap.get(AnalogInput.class, ANALOG_ENCODER_LEFT);
        analogRight = hardwareMap.get(AnalogInput.class, ANALOG_ENCODER_RIGHT);

        setTargetAngle(TargetAngle.UP);
        setTargetTwist(TargetTwist.NORMAL);
        setTargetState(TargetState.CLOSED_NORMAL);
    }


    public void setTargetAngle(TargetAngle targetAngle) {
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

    public TargetAngle getTargetAngle() {
        return targetAngle;
    }


    public void setTargetTwist(TargetTwist twistAngle) {
        switch (twistAngle) {
            case NORMAL:
                twistServo.setPosition(twist_normal_pos);
                break;
            case FLIPPED:
                twistServo.setPosition(twist_flipped_pos);
                break;
        }
    }

    public void setTwistIncrement(double increment) {
        twistIncrement = increment;
    }


    public void setTargetState(TargetState targetState) {
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