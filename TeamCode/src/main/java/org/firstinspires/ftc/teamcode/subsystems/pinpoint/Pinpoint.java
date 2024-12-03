package org.firstinspires.ftc.teamcode.subsystems.pinpoint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class Pinpoint extends VLRSubsystem<Pinpoint> implements PinpointConfiguration {
    GoBildaPinpointDriver pinpoint;

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_DEVICE_NAME);

        pinpoint.setOffsets(X_OFFSET, Y_OFFSET);
        pinpoint.setEncoderResolution(ENCODER_TYPE);

        pinpoint.setEncoderDirections(X_DIR, Y_DIR);

        pinpoint.resetPosAndIMU();
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public Pose2D getVelocity() {
        return pinpoint.getVelocity();
    }

    public void resetIMU() {
        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
    }
    // 71.3406 fwd
    // 58.2938

    @Override
    public void periodic() {
        update();
    }
}
