package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface ClawConfiguration {
    double CLAW_OPEN_ANGLE = 55;
    double CLAW_CLOSED_ANGLE = 0;

    String LEFT_CLAW = "leftClaw";
    String RIGHT_CLAW = "rightClaw";

    int MIN_LEFT_CLAW_DEGREE = 0;
    int MAX_LEFT_CLAW_DEGREE = 180;
    int MIN_RIGHT_CLAW_DEGREE = 0;
    int MAX_RIGHT_CLAW_DEGREE = 180;
}
