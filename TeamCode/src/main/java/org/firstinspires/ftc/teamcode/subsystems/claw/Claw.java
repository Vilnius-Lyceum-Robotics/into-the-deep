package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class Claw extends VLRSubsystem<Claw> implements ClawConfiguration {
    ServoEx leftClaw;
    ServoEx rightClaw;

    boolean isOpen;

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        leftClaw = new SimpleServo(hardwareMap, LEFT_CLAW, MIN_LEFT_CLAW_DEGREE, MAX_LEFT_CLAW_DEGREE);
        leftClaw.setInverted(true);
        rightClaw = new SimpleServo(hardwareMap, RIGHT_CLAW, MIN_RIGHT_CLAW_DEGREE, MAX_RIGHT_CLAW_DEGREE);
    }

    public void open() {
        leftClaw.turnToAngle(CLAW_OPEN_ANGLE);
        rightClaw.turnToAngle(CLAW_OPEN_ANGLE);
        isOpen = true;
    }

    public void close() {
        leftClaw.turnToAngle(CLAW_CLOSED_ANGLE);
        rightClaw.turnToAngle(CLAW_CLOSED_ANGLE);
        isOpen = false;
    }

    public boolean isOpen() {
        return isOpen;
    }
}
