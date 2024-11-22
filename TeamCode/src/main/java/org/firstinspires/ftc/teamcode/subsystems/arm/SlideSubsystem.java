package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile.FeedforwardType.SINE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem.mapToRange;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;

@Config
public class SlideSubsystem extends VLRSubsystem<ArmSubsystem> implements SlideConfiguration {
    private DcMotorEx extensionMotor0;
    private DcMotorEx extensionMotor1;
    private DcMotorEx extensionMotor2;
    private DcMotorEx extensionEncoder;

    private MotionProfile motionProfile;


    public SlideSubsystem(HardwareMap hardwareMap){
        initialize(hardwareMap);
    }

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        extensionMotor0 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_0);
        extensionMotor1 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_1);
        extensionMotor2 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_2);
        extensionEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        motionProfile = new MotionProfile(telemetry, "SLIDE", ACCELERATION, DECELERATION, MAX_VELOCITY, p, i, d, FEED_FORWARD_GAIN, VELOCITY_GAIN, ACCELERATION_GAIN, SINE, true);
        motionProfile.enableTelemetry(true);
    }


    public void setTargetPosition(TargetPosition position){
        motionProfile.setCurrentTargetPosition(clamp(position.extension, MIN_POSITION, MAX_POSITION));
    }

    public void setTargetPosition(double position){
        position = mapToRange(position, 0, 1, MIN_POSITION, MAX_POSITION);
        motionProfile.setCurrentTargetPosition(clamp(position, MIN_POSITION, MAX_POSITION));
    }


    public double getPosition(){
        return extensionEncoder.getCurrentPosition();
    }


    public void periodic(double armAngleDegrees){
        motionProfile.updateCoefficients(ACCELERATION, DECELERATION, MAX_VELOCITY, p, i, d, VELOCITY_GAIN, ACCELERATION_GAIN);
        double power = motionProfile.getPower(extensionEncoder.getCurrentPosition(), armAngleDegrees);

        extensionMotor0.setPower(power);
        extensionMotor1.setPower(power);
        extensionMotor2.setPower(power);
    }

}
