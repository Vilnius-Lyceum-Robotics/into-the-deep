//package org.firstinspires.ftc.teamcode.OpModes;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.roboctopi.cuttlefish.utils.Direction;
//import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
//import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
//import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
//
//import org.firstinspires.ftc.teamcode.Utils.MotionProfile;
//
//
//@Config
//@TeleOp(name = "motionProfileTest")
//
//public class TestOpMode extends OpMode {
//    CuttleMotor motor1, motor2, motor3;
//    //CuttleMotor motor;
//    CuttleRevHub controlHub;
//    CuttleEncoder encoder;
//    CuttleRevHub expansionHub;
//
//    MotionProfile motionProfile;
//
//    public static TargetPosition targetPosition = TargetPosition.DOWN;
//    public static double positionUP = 110;
//    public static double positionDown = 0;
//
//
//    @Override
//    public void init() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        controlHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.CONTROL_HUB);
//        expansionHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.EXPANSION_HUB);
//
//        //motor = new CuttleMotor(expansionHub, 0);
//        //motor.setDirection(Direction.REVERSE);
//
//        motor1 = new CuttleMotor(expansionHub, 1);
//        motor2 = new CuttleMotor(expansionHub, 2);
//        motor3 = new CuttleMotor(expansionHub, 3);
//
//
//        encoder = new CuttleEncoder(controlHub, 0, 28);
//        encoder.setDirection(Direction.FORWARD);
//
//        motionProfile = new MotionProfile(telemetry);
//    }
//
//
//    @Override
//    public void loop() {
//        controlHub.pullBulkData();
//
//        if (gamepad1.triangle){
//            targetPosition = TargetPosition.UP;
//        } else if (gamepad1.cross) {
//            targetPosition = TargetPosition.DOWN;
//        }
//
//        //double encoderAngle = Math.toDegrees(encoder.getRotation());
//        double encoderAngle = encoder.getRotation();
//
//
//        switch (targetPosition){
//            case UP:
//                motionProfile.setTargetPosition(positionUP, encoderAngle);
//                break;
//            case DOWN:
//                motionProfile.setTargetPosition(positionDown, encoderAngle);
//                break;
//        }
//
//        telemetry.addData("encoder angle degrees: ", encoderAngle);
//        double power = motionProfile.getPower(encoderAngle);
//
//        motor1.setPower(power);
//        motor2.setPower(power);
//        motor3.setPower(power);
//
//    }
//
//    public enum TargetPosition{
//        UP,
//        DOWN
//    }
//}