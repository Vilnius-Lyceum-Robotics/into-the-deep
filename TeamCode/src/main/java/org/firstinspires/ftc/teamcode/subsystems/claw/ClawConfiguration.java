package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface ClawConfiguration {
    double CLAW_OPEN_ANGLE = 55;
    double CLAW_CLOSED_ANGLE = 0;

    String LEFT_CLAW = "leftClaw";
    String RIGHT_CLAW = "rightClaw";

    Integer MIN_LEFT_CLAW_DEGREE = 0;
    Integer MAX_LEFT_CLAW_DEGREE = 180;
    Integer MIN_RIGHT_CLAW_DEGREE = 0;
    Integer MAX_RIGHT_CLAW_DEGREE = 180;
}
