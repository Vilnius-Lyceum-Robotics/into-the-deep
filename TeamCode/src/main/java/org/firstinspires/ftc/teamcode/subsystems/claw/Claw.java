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
        leftClaw = new SimpleServo(hardwareMap, "leftClaw", 0, 180);
        leftClaw.setInverted(true);
        rightClaw = new SimpleServo(hardwareMap, "rightClaw", 0, 180);
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
