package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;


@Config
@Disabled
@TeleOp(name="axonMicroCalibration")
public class axonCalibration extends OpMode
{
    private CuttleServo servo;
    private CuttleServo servo2;
    private CuttleRevHub controlHub;
    public static double position = 0;
    public static double position1 = 0;

    @Override
    public void init() {
        controlHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.CONTROL_HUB);
        servo = new CuttleServo(controlHub, 0);
        servo2 = new CuttleServo(controlHub, 1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        servo.setPosition(position);
        servo2.setPosition(position1);
    }
}
