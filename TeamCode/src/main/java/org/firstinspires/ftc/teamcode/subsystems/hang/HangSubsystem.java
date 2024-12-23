package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;


public class HangSubsystem extends VLRSubsystem<HangSubsystem> implements HangConfiguration {
    private CRServo leftServo, rightServo;

    protected void initialize(HardwareMap hardwareMap) {
       leftServo = hardwareMap.get(CRServo.class, LEFT_SERVO);
       rightServo = hardwareMap.get(CRServo.class, RIGHT_SERVO);

       leftServo.setDirection(DcMotorEx.Direction.FORWARD);
       rightServo.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void setPower (double power){
        leftServo.setPower(power);
        rightServo.setPower(power);
    }
}