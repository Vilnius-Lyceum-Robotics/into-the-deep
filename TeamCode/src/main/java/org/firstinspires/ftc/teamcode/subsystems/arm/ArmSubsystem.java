package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile.FeedforwardType.COSINE;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

@Config
public class ArmSubsystem extends VLRSubsystem<ArmSubsystem> implements ArmConfiguration{
    private DcMotorEx motor;
    private DcMotorEx thoughBoreEncoder;

    private MotionProfile motionProfile;
    private SlideSubsystem slideSubsystem;


    protected void initialize(HardwareMap hardwareMap) {
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        slideSubsystem = new SlideSubsystem(hardwareMap, telemetry);

        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        thoughBoreEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);

        motionProfile = new MotionProfile(telemetry, acceleration, deceleration, maxVelocity, p, i, d, v, a, COSINE);
        motionProfile.setTelemetryName("ARM");
        motionProfile.enableTelemetry(true);
    }


    public void setTargetAngle(TargetAngle targetAngle){
        motionProfile.setTargetPosition(clamp(targetAngle.angleDegrees, minAngle, maxAngle));
    }


    public void setTargetPosition(SlideConfiguration.TargetPosition targetPosition){
        slideSubsystem.setTargetPosition(targetPosition);
    }


    public void setTargetPosition(double targetPosition){
        slideSubsystem.setTargetPosition(targetPosition);
    }


    public double getAngleDegrees(){
        return thoughBoreEncoder.getCurrentPosition() / encoderTicksPerRotation * 360d;
    }


    @Override
    public void periodic(){
        motionProfile.updateCoefficients(acceleration, deceleration, maxVelocity, p, i, d, v, a);

        double currentAngle = getAngleDegrees();
        double f_current = map(slideSubsystem.getPosition(), SlideSubsystem.minPosition, SlideSubsystem.maxPosition, f_retracted, f_extended);

        motionProfile.set_f(f_current);

        double power = motionProfile.getPower(currentAngle);
        motor.setPower(power);

        slideSubsystem.periodic(currentAngle);
    }


    public static double map(double value, double inMIN, double inMAX, double outMIN, double outMAX){
        if (inMIN == inMAX) {
            throw new IllegalArgumentException("inMIN and inMax cant be the same");
        }
        return outMIN + ((value - inMIN) * (outMAX - outMIN)) / (inMAX - inMIN);
    }
}
