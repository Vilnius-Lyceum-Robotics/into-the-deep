package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface LiftConfiguration {
    int LIFT_UP_POS = 5500;
    int LIFT_DOWN_POS = 0;

    int LIFT_POS_TOLERANCE = 10;
}
