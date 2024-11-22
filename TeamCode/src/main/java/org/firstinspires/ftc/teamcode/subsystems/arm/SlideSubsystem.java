package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile.FeedforwardType.SINE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem.map;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;

@Config
public class SlideSubsystem implements SlideConfiguration {
    private DcMotorEx extensionMotor0;
    private DcMotorEx extensionMotor1;
    private DcMotorEx extensionMotor2;
    private DcMotorEx extensionEncoder;

    private MotionProfile motionProfile;


    public SlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        extensionMotor0 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_0);
        extensionMotor1 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_1);
        extensionMotor2 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_2);
        extensionEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);

        motionProfile = new MotionProfile(telemetry, acceleration, deceleration, maxVelocity, p, i, d, f, v, a, SINE);

        motionProfile.setTelemetryName("SLIDE");
        motionProfile.enableTelemetry(true);
    }


    public void setTargetPosition(TargetPosition position){
        motionProfile.setTargetPosition(clamp(position.extension, minPosition, maxPosition));
    }

    public void setTargetPosition(double position){
        position = map(position, 0, 1, minPosition, maxPosition);
        motionProfile.setTargetPosition(clamp(position, minPosition, maxPosition));
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
