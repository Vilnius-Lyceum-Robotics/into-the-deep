package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile.FeedforwardType.SINE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem.mapToRange;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.ACCELERATION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.ACCELERATION_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.DECELERATION_FAST;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.ENCODER_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.FEEDBACK_DERIVATIVE_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.FEEDBACK_INTEGRAL_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.FEEDBACK_PROPORTIONAL_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.FEED_FORWARD_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.MAX_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.MAX_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.MIN_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.MOTOR_NAME_0;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.MOTOR_NAME_1;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.MOTOR_NAME_2;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.TargetPosition;
import static org.firstinspires.ftc.teamcode.subsystems.arm.SlideConfiguration.VELOCITY_GAIN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;

@Config
public class SlideSubsystem extends VLRSubsystem<ArmSubsystem> {
    private DcMotorEx extensionMotor0;
    private DcMotorEx extensionMotor1;
    private DcMotorEx extensionMotor2;
    private DcMotorEx extensionEncoder;

    private MotionProfile motionProfile;


    public SlideSubsystem(HardwareMap hardwareMap) {
        initialize(hardwareMap);
    }

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        extensionMotor0 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_0);
        extensionMotor1 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_1);
        extensionMotor2 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_2);

        extensionMotor0.setDirection(DcMotorEx.Direction.FORWARD);
        extensionMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        extensionMotor2.setDirection(DcMotorEx.Direction.FORWARD);

        extensionEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        extensionEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extensionEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        motionProfile = new MotionProfile(telemetry, "SLIDE", ACCELERATION, DECELERATION_FAST, MAX_VELOCITY, FEEDBACK_PROPORTIONAL_GAIN, FEEDBACK_INTEGRAL_GAIN, FEEDBACK_DERIVATIVE_GAIN, FEED_FORWARD_GAIN, VELOCITY_GAIN, ACCELERATION_GAIN, SINE, true);
        motionProfile.enableTelemetry(true);
    }


    public void setTargetPosition(TargetPosition position) {
        setTargetPosition(position.extension);
    }

    public void setTargetPosition(double position) {
        position = mapToRange(position, 0, 1, MIN_POSITION, MAX_POSITION);
        motionProfile.setCurrentTargetPosition(clamp(position, MIN_POSITION, MAX_POSITION));
    }


    public double getPosition() {
        return extensionEncoder.getCurrentPosition();
    }


    public void periodic(double armAngleDegrees) {
        motionProfile.updateCoefficients(ACCELERATION, DECELERATION_FAST, MAX_VELOCITY, FEEDBACK_PROPORTIONAL_GAIN, FEEDBACK_INTEGRAL_GAIN, FEEDBACK_DERIVATIVE_GAIN, VELOCITY_GAIN, ACCELERATION_GAIN);
        motionProfile.setFeedForwardGain(FEED_FORWARD_GAIN);
        double power = motionProfile.getPower(extensionEncoder.getCurrentPosition(), armAngleDegrees);

        extensionMotor0.setPower(power);
        extensionMotor1.setPower(power);
        extensionMotor2.setPower(power);
    }

}
