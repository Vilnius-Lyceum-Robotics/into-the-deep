package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile.FeedforwardType.COSINE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class ArmSubsystem extends VLRSubsystem<ArmSubsystem> {
    private DcMotorEx motor;
    private DcMotorEx thoughBoreEncoder;

    private MotionProfile motionProfile;
    private SlideSubsystem slideSubsystem;


    protected void initialize(HardwareMap hardwareMap) {
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        slideSubsystem = new SlideSubsystem(hardwareMap);

        motor = hardwareMap.get(DcMotorEx.class, ArmConfiguration.MOTOR_NAME);
        motor.setDirection(DcMotorEx.Direction.REVERSE);

        thoughBoreEncoder = hardwareMap.get(DcMotorEx.class, ArmConfiguration.ENCODER_NAME);
        thoughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        thoughBoreEncoder.setDirection(DcMotorEx.Direction.FORWARD);

        motionProfile = new MotionProfile(telemetry, "ARM", ArmConfiguration.ACCELERATION, ArmConfiguration.DECELERATION, ArmConfiguration.MAX_VELOCITY, ArmConfiguration.FEEDBACK_PROPORTIONAL_GAIN, ArmConfiguration.FEEDBACK_INTEGRAL_GAIN, ArmConfiguration.FEEDBACK_DERIVATIVE_GAIN, ArmConfiguration.VELOCITY_GAIN, ArmConfiguration.ACCELERATION_GAIN, COSINE, true);
        motionProfile.enableTelemetry(true);
    }


    public void setTargetAngle(ArmConfiguration.TargetAngle targetAngle) {
        motionProfile.setCurrentTargetPosition(clamp(targetAngle.angleDegrees, ArmConfiguration.MIN_ANGLE, ArmConfiguration.MAX_ANGLE));
    }


    public void setTargetPosition(SlideConfiguration.TargetPosition targetPosition) {
        slideSubsystem.setTargetPosition(targetPosition);
    }


    public void setTargetPosition(double targetPosition) {
        slideSubsystem.setTargetPosition(targetPosition);
    }


    public double getAngleDegrees() {
        return thoughBoreEncoder.getCurrentPosition() / ArmConfiguration.ENCODER_TICKS_PER_ROTATION * 360d;
    }


    @Override
    public void periodic() {
        motionProfile.updateCoefficients(ArmConfiguration.ACCELERATION, ArmConfiguration.DECELERATION, ArmConfiguration.MAX_VELOCITY, ArmConfiguration.FEEDBACK_PROPORTIONAL_GAIN, ArmConfiguration.FEEDBACK_INTEGRAL_GAIN, ArmConfiguration.FEEDBACK_DERIVATIVE_GAIN, ArmConfiguration.VELOCITY_GAIN, ArmConfiguration.ACCELERATION_GAIN);

        double currentAngle = getAngleDegrees();
        double currentFeedForwardGain = mapToRange(slideSubsystem.getPosition(), SlideConfiguration.MIN_POSITION, SlideConfiguration.MAX_POSITION, ArmConfiguration.RETRACTED_FEEDFORWARD_GAIN, ArmConfiguration.EXTENDED_FEEDFORWARD_GAIN);

        motionProfile.setFeedForwardGain(currentFeedForwardGain);

        double power = motionProfile.getPower(currentAngle);
        motor.setPower(power);

        slideSubsystem.periodic(currentAngle);
    }


    public static double mapToRange(double value, double minInput, double maxInput, double minOutput, double maxOutput) {
        if (minInput == maxInput) {
            throw new IllegalArgumentException("inMIN and inMax cant be the same");
        }
        return minOutput + ((value - minInput) * (maxOutput - minOutput)) / (maxInput - minInput);
    }
}
