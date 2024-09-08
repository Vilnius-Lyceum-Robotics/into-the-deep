package org.firstinspires.ftc.teamcode.helpers.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class VLRLinearOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        this.run();
    }

    public abstract void run();
}
