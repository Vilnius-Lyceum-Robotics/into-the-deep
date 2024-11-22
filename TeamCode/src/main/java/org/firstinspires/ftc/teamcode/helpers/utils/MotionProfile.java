package org.firstinspires.ftc.teamcode.helpers.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotionProfile {
    private Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();

    private double p, i, d, f, v, a;
    private double acceleration, deceleration, maxVelocity;

    private double targetPosition = 0;
    private double prevTargetPosition = 0;
    private double startingPosition = 0;

    private String telemetryName = "";
    private boolean enableTelemetry = false;

    private FeedforwardType feedforwardType = FeedforwardType.CONSTANT;
    private PIDController pid = new PIDController(p, i, d);



    public MotionProfile(Telemetry telemetry, double acceleration, double deceleration, double maxVelocity, double p, double i, double d, double f, double v, double a, FeedforwardType feedforwardType){
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.acceleration = acceleration;
        this.deceleration = deceleration;
        this.maxVelocity = maxVelocity;
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.v = v;
        this.a = a;
        this.feedforwardType = feedforwardType;
    }

    public MotionProfile(Telemetry telemetry, double acceleration, double deceleration, double maxVelocity, double p, double i, double d, double v, double a, FeedforwardType feedforwardType){
        this(telemetry, acceleration, deceleration, maxVelocity, p, i, d, 0, v, a, feedforwardType);
    }

    public void updateCoefficients(double acceleration, double deceleration, double maxVelocity, double p, double i, double d, double v, double a){
        this.acceleration = acceleration;
        this.deceleration = deceleration;
        this.maxVelocity = maxVelocity;
        this.p = p;
        this.i = i;
        this.d = d;
        this.v = v;
        this.a = a;
        pid.setPID(p, i, d);
    }

    public void set_f(double f){
        this.f = f;
    }


    public void setTargetPosition(double targetPosition){
        this.targetPosition = targetPosition;
    }


    public double getPower(double currentPosition, double feedforwardAngle){
        if (targetPosition != prevTargetPosition) {
            prevTargetPosition = targetPosition;
            startingPosition = currentPosition;
            timer.reset();
        }

        double positionError = targetPosition - startingPosition;
        double [] setPoints = getSetPoints(Math.abs(positionError), timer.seconds());

        double positionSetPoint = startingPosition + Math.signum(positionError) * setPoints[0];
        double velocitySetPoint = setPoints[1];
        double accelerationSetPoint = setPoints[2];

        double positionPower = pid.calculate(currentPosition, positionSetPoint);
        double velocityPower = velocitySetPoint * v * Math.signum(positionError);
        double accelerationPower = accelerationSetPoint * a * Math.signum(positionError);

        double feedforward = 0;
        switch (feedforwardType){
            case SINE:
                feedforward = Math.sin(Math.toRadians(feedforwardAngle)) * f;
                break;
            case COSINE:
                feedforward = Math.cos(Math.toRadians(feedforwardAngle)) * f;
                break;
            case CONSTANT:
                feedforward = f;
                break;
            case NO_FEEDFORWARD:
                break;
        }

        if (enableTelemetry) {
            telemetry.addData(telemetryName + "_motionProfileCurrentPosition: ", currentPosition);
            telemetry.addData(telemetryName + "_motionProfileFeedforwardAngle: ", feedforwardAngle);
            telemetry.addData(telemetryName + "_motionProfileTargetPosition: ", positionSetPoint);
            telemetry.addData(telemetryName + "_motionProfileTargetVelocity: ", velocitySetPoint);
            telemetry.addData(telemetryName + "_motionProfileTargetAcceleration: ", accelerationSetPoint);
            telemetry.addData(telemetryName + "_motionProfileTime: ", timer.seconds());
            telemetry.addData(telemetryName + "_motor power: ", positionPower + velocityPower + accelerationPower);
        }

        return positionPower + velocityPower + accelerationPower + feedforward;
    }


    public double getPower(double currentPosition){
        return getPower(currentPosition, currentPosition);
    }


    private double [] getSetPoints(double distance, double elapsed_time) {
        if (distance == 0) return new double[] {0, 0, 0};
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
            return new double[] {distance, 0, 0};
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return new double[] {0.5 * acceleration * Math.pow(elapsed_time, 2), acceleration * elapsed_time, acceleration};
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return new double[] {acceleration_distance + maxAchievableVelocity * cruise_current_dt, maxAchievableVelocity, 0};
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = maxAchievableVelocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return new double[] {acceleration_distance + cruise_distance + maxAchievableVelocity * deceleration_time - 0.5 * deceleration * Math.pow(deceleration_time, 2),
                    maxAchievableVelocity - deceleration * deceleration_time, deceleration};
        }
    }

    public void setTelemetryName(String telemetryName){
        this.telemetryName = telemetryName;
    }

    public void enableTelemetry(boolean enableTelemetry){
        this.enableTelemetry = enableTelemetry;
    }

    public enum FeedforwardType{
        SINE,
        COSINE,
        CONSTANT,
        NO_FEEDFORWARD
    }
}