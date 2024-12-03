package org.firstinspires.ftc.teamcode.subsystems.pinpoint;

public interface PinpointConfiguration {
    String PINPOINT_DEVICE_NAME = "pinpoint";

    double X_OFFSET = -80; // offset in mm from pinpoint, left is +
    double Y_OFFSET = 1; // offset in mm from pinpoint, forward is +

    GoBildaPinpointDriver.EncoderDirection X_DIR = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    GoBildaPinpointDriver.EncoderDirection Y_DIR = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    GoBildaPinpointDriver.GoBildaOdometryPods ENCODER_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;


}
