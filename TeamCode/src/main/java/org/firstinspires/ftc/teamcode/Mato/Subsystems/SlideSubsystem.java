package org.firstinspires.ftc.teamcode.Mato.Subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile.FeedforwardType.SINE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;

@Config
public class SlideSubsystem {
    public static double acceleration = 4500;
    public static double deceleration = 2000;
    public static double maxVelocity = 400;
    public static double p = 0.025;
    public static double i = 0;
    public static double d = 0.002;
    public static double f = 0;
    public static double v = 0.0012;
    public static double a = 0.00012;

    public static double minPosition = 0;
    public static double maxPosition = 270;

    public static double position_intake = 0.2;
    public static double position_deposit = 0.9;

    private DcMotorEx extensionMotor0;
    private DcMotorEx extensionMotor1;
    private DcMotorEx extensionMotor2;
    private DcMotorEx extensionEncoder;

    private MotionProfile motionProfile;


    public SlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        extensionMotor0 = hardwareMap.get(DcMotorEx.class, "EH1");
        extensionMotor1 = hardwareMap.get(DcMotorEx.class, "EH2");
        extensionMotor2 = hardwareMap.get(DcMotorEx.class, "EH3");
        extensionEncoder = hardwareMap.get(DcMotorEx.class, "CH0");

        motionProfile = new MotionProfile(telemetry, acceleration, deceleration, maxVelocity, p, i, d, f, v, a, SINE);

        motionProfile.setTelemetryName("EXTENSION");
        motionProfile.enableTelemetry(true);
    }


    public void setTargetPosition(double targetPosition){
        motionProfile.setTargetPosition(clamp(targetPosition, minPosition, maxPosition));
    }


    public double getPosition(){
        return extensionEncoder.getCurrentPosition();
    }


    public void periodic(double armAngleDegrees){
        motionProfile.updateCoefficients(acceleration, deceleration, maxVelocity, p, i, d, v, a);
        double power = motionProfile.getPower(extensionEncoder.getCurrentPosition(), armAngleDegrees);

        extensionMotor0.setPower(power);
        extensionMotor1.setPower(power);
        extensionMotor2.setPower(power);
    }
}
