package org.firstinspires.ftc.teamcode.Mato.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Mato.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;


@Config
@TeleOp(name = "subsystemTest")

public class SubsystemTest extends VLRLinearOpMode {
    ArmSubsystem armSubsystem;
    public static ArmSubsystem.STATE state = ArmSubsystem.STATE.DOWN;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(ArmSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        armSubsystem = VLRSubsystem.getInstance(ArmSubsystem.class);

        waitForStart();

        while(opModeIsActive()){
            armSubsystem.setState(state);
        }
    }
}
