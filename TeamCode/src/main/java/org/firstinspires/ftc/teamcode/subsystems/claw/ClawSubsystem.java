package org.firstinspires.ftc.teamcode.subsystems.claw;

import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.ANALOG_ENCODER_NAME_0;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.ANALOG_ENCODER_NAME_1;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.SERVO_NAME_0;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.SERVO_NAME_1;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.SERVO_NAME_2;
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


    protected void initialize(HardwareMap hardwareMap) {
        angleServo = hardwareMap.get(Servo.class, SERVO_NAME_0);
        twistServo = hardwareMap.get(Servo.class, SERVO_NAME_1);
        grabServos = hardwareMap.get(Servo.class, SERVO_NAME_2);

        analogLeft = hardwareMap.get(AnalogInput.class, ANALOG_ENCODER_NAME_0);
        analogRight = hardwareMap.get(AnalogInput.class, ANALOG_ENCODER_NAME_1);
    }


    public void setTargetAngle(ClawConfiguration.TargetAngle targetAngle) {
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
    }


    public boolean isSamplePresent() {
        return analogLeft.getVoltage() > analog_voltage_left && analogRight.getVoltage() > analog_voltage_right;
    }
}