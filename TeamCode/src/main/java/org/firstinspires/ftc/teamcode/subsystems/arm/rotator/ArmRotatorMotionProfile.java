package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


// TODO: seperate config
@Config
public class ArmRotatorMotionProfile {
    Telemetry telemetry;
    ElapsedTime timer = new ElapsedTime();

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;
    public static double v = 0;
    public static double a = 0;

    public static double acceleration = 0;
    public static double deceleration = 0;
    public static double maxVelocity = 0;

    private double targetPosition = 0;
    private double prevTargetPosition = 0;
    private double startingPosition = 0;

    private double lastPosition;
    private final PIDController pid = new PIDController(p, i, d);


    public ArmRotatorMotionProfile(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * Constructor for the ArmRotatorMotionProfile class, without DS telemetry
     */
    public ArmRotatorMotionProfile() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    public void setTargetPosition(double targetPosition) {
        if (targetPosition != prevTargetPosition) {
            this.targetPosition = targetPosition;
            prevTargetPosition = targetPosition;
        }
        startingPosition = lastPosition;
        timer.reset();
    }

    public void update(double angle) {
        lastPosition = angle;
    }

    public double getPower() {
        pid.setPID(p, i, d);

        double positionError = targetPosition - startingPosition;
        double[] setPoints = getSetPoints(Math.abs(positionError), timer.seconds());

        double positionSetPoint = startingPosition + Math.signum(positionError) * setPoints[0];
        double velocitySetPoint = setPoints[1];
        double accelerationSetPoint = setPoints[2];

        double positionPower = pid.calculate(lastPosition, positionSetPoint) + Math.sin(Math.toRadians(lastPosition)) * f;
        double velocityPower = velocitySetPoint * v * Math.signum(positionError);
        double accelerationPower = accelerationSetPoint * a * Math.signum(positionError);

        telemetry.addData("encoder angle degrees: ", lastPosition);
        telemetry.addData("motionProfileCurrentPosition: ", lastPosition);
        telemetry.addData("motionProfileInstantaneousTargetPosition: ", positionSetPoint);
        telemetry.addData("motionProfileTargetPosition: ", positionSetPoint);
        telemetry.addData("motionProfileTargetVelocity: ", velocitySetPoint);
        telemetry.addData("motionProfileTime: ", timer.seconds());
        telemetry.addData("motor power: ", positionPower + velocityPower + accelerationPower);

        return positionPower + velocityPower + accelerationPower;
    }


    double[] getSetPoints(double distance, double elapsed_time) {
        if (distance == 0) return new double[]{0, 0, 0};
        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = maxVelocity / acceleration;
        double deceleration_dt = maxVelocity / deceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);
        double deceleration_distance = 0.5 * deceleration * Math.pow(deceleration_dt, 2);

        if (acceleration_distance + deceleration_distance > distance) {
            acceleration_dt = Math.sqrt(2 * distance / (acceleration * (1 + acceleration / deceleration)));
        }

        acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        double maxAchievableVelocity = acceleration * acceleration_dt;

        deceleration_dt = maxAchievableVelocity / deceleration;
        deceleration_distance = 0.5 * deceleration * Math.pow(deceleration_dt, 2);

        // calculate the time that we're at max velocity
        double cruise_distance = distance - acceleration_distance - deceleration_distance;
        double cruise_dt = cruise_distance / maxAchievableVelocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            return new double[]{distance, 0, 0};
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return new double[]{0.5 * acceleration * Math.pow(elapsed_time, 2), acceleration * elapsed_time, acceleration};
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return new double[]{acceleration_distance + maxAchievableVelocity * cruise_current_dt, maxAchievableVelocity, 0};
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = maxAchievableVelocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return new double[]{acceleration_distance + cruise_distance + maxAchievableVelocity * deceleration_time - 0.5 * deceleration * Math.pow(deceleration_time, 2),
                    maxAchievableVelocity - deceleration * deceleration_time, deceleration};
        }
    }
}