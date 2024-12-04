package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile.FeedforwardType.COSINE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;

public class ArmSubsystem extends VLRSubsystem<ArmSubsystem> {
    private DcMotorEx motor;
    private DcMotorEx thoughBoreEncoder;

    private MotionProfile motionProfile;
    private SlideSubsystem slideSubsystem;

    private ArmState state = ArmState.IN_ROBOT;

    public static double mapToRange(double value, double minInput, double maxInput, double minOutput, double maxOutput) {
        if (minInput == maxInput) {
            throw new IllegalArgumentException("inMIN and inMax cant be the same");
        }
        return minOutput + ((value - minInput) * (maxOutput - minOutput)) / (maxInput - minInput);
    }

    protected void initialize(HardwareMap hardwareMap) {
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        slideSubsystem = VLRSubsystem.getInstance(SlideSubsystem.class);

        motor = hardwareMap.get(DcMotorEx.class, ArmRotatingPartConfiguration.MOTOR_NAME);
        motor.setDirection(DcMotorEx.Direction.REVERSE);

        thoughBoreEncoder = hardwareMap.get(DcMotorEx.class, ArmRotatingPartConfiguration.ENCODER_NAME);
        thoughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //thoughBoreEncoder.setDirection(DcMotorEx.Direction.REVERSE);

        motionProfile = new MotionProfile(telemetry, "ARM", ArmRotatingPartConfiguration.ACCELERATION, ArmRotatingPartConfiguration.DECELERATION, ArmRotatingPartConfiguration.MAX_VELOCITY, ArmRotatingPartConfiguration.FEEDBACK_PROPORTIONAL_GAIN, ArmRotatingPartConfiguration.FEEDBACK_INTEGRAL_GAIN, ArmRotatingPartConfiguration.FEEDBACK_DERIVATIVE_GAIN, ArmRotatingPartConfiguration.VELOCITY_GAIN, ArmRotatingPartConfiguration.ACCELERATION_GAIN, COSINE, true);
        motionProfile.enableTelemetry(true);
    }

    public void setTargetAngle(ArmRotatingPartConfiguration.TargetAngle targetAngle) {
        motionProfile.setCurrentTargetPosition(clamp(targetAngle.angleDegrees, ArmRotatingPartConfiguration.MIN_ANGLE, ArmRotatingPartConfiguration.MAX_ANGLE));
    }

    public void setTargetPosition(SlideConfiguration.TargetPosition targetPosition) {
        slideSubsystem.setTargetPosition(targetPosition);
    }

    public void setTargetPosition(double targetPosition) {
        slideSubsystem.setTargetPosition(targetPosition);
    }

    public double getAngleDegrees() {
        return thoughBoreEncoder.getCurrentPosition() / ArmRotatingPartConfiguration.ENCODER_TICKS_PER_ROTATION * 360d;
    }

    @Override
    public void periodic() {
        motionProfile.updateCoefficients(ArmRotatingPartConfiguration.ACCELERATION, ArmRotatingPartConfiguration.DECELERATION, ArmRotatingPartConfiguration.MAX_VELOCITY, ArmRotatingPartConfiguration.FEEDBACK_PROPORTIONAL_GAIN, ArmRotatingPartConfiguration.FEEDBACK_INTEGRAL_GAIN, ArmRotatingPartConfiguration.FEEDBACK_DERIVATIVE_GAIN, ArmRotatingPartConfiguration.VELOCITY_GAIN, ArmRotatingPartConfiguration.ACCELERATION_GAIN);

        double currentAngle = getAngleDegrees();
        double currentFeedForwardGain = mapToRange(slideSubsystem.getPosition(), SlideConfiguration.MIN_POSITION, SlideConfiguration.MAX_POSITION, ArmRotatingPartConfiguration.RETRACTED_FEEDFORWARD_GAIN, ArmRotatingPartConfiguration.EXTENDED_FEEDFORWARD_GAIN);

        motionProfile.setFeedForwardGain(currentFeedForwardGain);

        double power = motionProfile.getPower(currentAngle);
        motor.setPower(power);

        slideSubsystem.periodic(currentAngle);
    }

    public enum ArmState {
        IN_ROBOT,
        PRE_INTAKE,
        INTAKE,
        SECOND_BASKET,
        FIRST_BASKET

    }

    public ArmState getState() {
        return this.state;
    }

    public void setState(ArmState state) {
        this.state = state;
    }
}
