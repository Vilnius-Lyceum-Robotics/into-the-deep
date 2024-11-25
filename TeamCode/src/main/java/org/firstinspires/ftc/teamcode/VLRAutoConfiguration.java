package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;

public interface VLRAutoConfiguration {
    // All measurements here are in inches.
    double PLATE = 24;

    double ROBOT_L = 45.5 / 2.54;
    double ROBOT_W = 32.3 / 2.54;

    enum StartPose {
        BLUE_RIGHT(new Pose(-3 * PLATE + ROBOT_L / 2, -1 * PLATE + ROBOT_W / 2, 0));

        public final Pose pose;

        StartPose(Pose pose) {
            this.pose = pose;
        }
    }
}
