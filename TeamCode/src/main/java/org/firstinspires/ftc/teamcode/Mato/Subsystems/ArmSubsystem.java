package org.firstinspires.ftc.teamcode.Mato.Subsystems;

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
public class ArmSubsystem extends VLRSubsystem<ArmSubsystem> {
    public static double acceleration = 6000;
    public static double deceleration = 2100;
    public static double maxVelocity = 400;

    public static double p = 0.03;
    public static double i = 0;
    public static double d = 0.0028;
    public static double v = 0.003;
    public static double a = 0.00016;

    public static double f_retracted = 0.08;
    public static double f_extended = 0.12;

    public static double minAngle = 0;
    public static double maxAngle = 150;

    private double encoderTicksPerRotation = 8192;

    public static double armAngle_UP = 125;

    private DcMotorEx motor;
    private DcMotorEx thoughBoreEncoder;

    private MotionProfile motionProfile;
    private SlideSubsystem slideSubsystem;


    protected void initialize(HardwareMap hardwareMap) {
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        slideSubsystem = new SlideSubsystem(hardwareMap, telemetry);

        motor = hardwareMap.get(DcMotorEx.class, "EH0");
        thoughBoreEncoder = hardwareMap.get(DcMotorEx.class, "EH3");

        motionProfile = new MotionProfile(telemetry, acceleration, deceleration, maxVelocity, p, i, d, v, a, COSINE);
        motionProfile.setTelemetryName("ARM");
        motionProfile.enableTelemetry(true);
    }


    public void setState(STATE state){
        switch (state){
            case DOWN:
                setTarget(0, 0);
                break;
            case INTAKE:
                setTarget(0, SlideSubsystem.position_intake);
                break;
            case DEPOSIT:
                setTarget(armAngle_UP, SlideSubsystem.position_deposit);
                break;
        }
    }


    public void setTarget(double armAngleDegrees, double slideExtension_from_0_to_1){
        motionProfile.setTargetPosition(clamp(armAngleDegrees, minAngle, maxAngle));
        slideSubsystem.setTargetPosition(map(slideExtension_from_0_to_1, 0, 1, minAngle, maxAngle));
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


    public double map(double value, double inMIN, double inMAX, double outMIN, double outMAX){
        if (inMIN == inMAX) {
            throw new IllegalArgumentException("inMIN and inMax cant be the same");
        }
        return outMIN + ((value - inMIN) * (outMAX - outMIN)) / (inMAX - inMIN);
    }


    public enum STATE {
        DOWN,
        INTAKE,
        DEPOSIT
    }
}
