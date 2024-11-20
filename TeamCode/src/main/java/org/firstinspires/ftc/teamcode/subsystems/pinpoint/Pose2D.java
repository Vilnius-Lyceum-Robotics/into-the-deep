package org.firstinspires.ftc.teamcode.subsystems.pinpoint;

/**
 * Represents a 2D pose in the field coordinate system.
 */
public class Pose2D {
    /**
     * The x coordinate of the pose in mm.
     */
    double x; // mm
    /**
     * The y coordinate of the pose in mm.
     */
    double y; // mm
    /**
     * The heading of the pose in radians.
     */
    double heading; // rad

    public Pose2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}
