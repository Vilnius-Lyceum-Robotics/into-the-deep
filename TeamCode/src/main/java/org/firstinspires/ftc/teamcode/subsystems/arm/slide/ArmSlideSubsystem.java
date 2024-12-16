package org.firstinspires.ftc.teamcode.subsystems.arm.slide;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile.FeedforwardType.SINE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem.mapToRange;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;

@Config
public class ArmSlideSubsystem extends VLRSubsystem<ArmSlideSubsystem> {
    private DcMotorEx extensionMotor0;
    private DcMotorEx extensionMotor1;
    private DcMotorEx extensionMotor2;
    private DcMotorEx extensionEncoder;

    private MotionProfile motionProfile;

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        extensionMotor0 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_0);
        extensionMotor1 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_1);
        extensionMotor2 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_2);

        extensionMotor0.setDirection(DcMotorEx.Direction.FORWARD);
        extensionMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        extensionMotor2.setDirection(DcMotorEx.Direction.FORWARD);

        extensionMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extensionMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extensionMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        extensionEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        extensionEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extensionEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        motionProfile = new MotionProfile(telemetry, "SLIDE", ACCELERATION, DECELERATION_FAST, MAX_VELOCITY, CREEP, FEEDBACK_PROPORTIONAL_GAIN, FEEDBACK_INTEGRAL_GAIN, FEEDBACK_DERIVATIVE_GAIN, FEED_FORWARD_GAIN, VELOCITY_GAIN, ACCELERATION_GAIN, SINE);
        motionProfile.enableTelemetry(true);
    }


    public void setTargetPosition(TargetPosition position) {
        setTargetPosition(position.extension);
    }

    public void setTargetPosition(double position) {
        position = mapToRange(position, 0, 1, MIN_POSITION, MAX_POSITION);
        motionProfile.setCurrentTargetPosition(clamp(position, MIN_POSITION, MAX_POSITION));
    }

    public boolean reachedTargetPosition() {
        return reachedPosition(getTargetPosition());
    }

    public boolean reachedPosition(double position) {
        return Math.abs(getPosition() - position) < ERROR_MARGIN;
    }

    public double getPosition() {
        return -extensionEncoder.getCurrentPosition();
    }

    public double getTargetPosition() {
        return motionProfile.getTargetPosition();
    }

    public double getTargetExtension(){return mapToRange(getPosition(), MIN_POSITION, MAX_POSITION, 0, 1);}

    public void incrementTargetPosition(double increment) {
        motionProfile.setCurrentTargetPosition(clamp(getTargetPosition() + increment, MIN_POSITION, HORIZONTAL_EXTENSION_LIMIT));
    }


    public void periodic(double armAngleDegrees) {
        motionProfile.updateCoefficients(ACCELERATION, DECELERATION_FAST, MAX_VELOCITY, FEEDBACK_PROPORTIONAL_GAIN, FEEDBACK_INTEGRAL_GAIN, FEEDBACK_DERIVATIVE_GAIN, VELOCITY_GAIN, ACCELERATION_GAIN);
        motionProfile.setFeedForwardGain(FEED_FORWARD_GAIN);

        if (getPosition() > HORIZONTAL_EXTENSION_LIMIT && armAngleDegrees < 90){
            setTargetPosition(HORIZONTAL_EXTENSION_LIMIT);
        }

        double power = motionProfile.getPower(getPosition(), armAngleDegrees);

        if (reachedPosition(TargetPosition.RETRACTED.extension) && getTargetExtension() == TargetPosition.RETRACTED.extension){
            extensionMotor1.setPower(0);
            extensionMotor2.setPower(0);
            extensionMotor0.setPower(0);

        }
        else{
            extensionMotor0.setPower(power);
            extensionMotor1.setPower(power);
            extensionMotor2.setPower(power);
        }
    }
}