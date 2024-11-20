package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.commands.CommandRunner;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Photon
@Autonomous(name = "VLRAuto")
public class VLRAuto extends VLRLinearOpMode {
    // Execution
    ExecutorService executorService;
    // Commands
    CommandRunner commandRunner;

    CommandScheduler cs;

    Follower d;

    final double PLATE = 24; // in

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(Chassis.class, Pinpoint.class);
        VLRSubsystem.initializeAll(hardwareMap);

        executorService = Executors.newCachedThreadPool();

        commandRunner = new CommandRunner(this::opModeIsActive);
        executorService.submit(commandRunner);
        cs = CommandScheduler.getInstance();

        d = new Follower(hardwareMap);

        Pose startingPose = new Pose(0, 0, 0); // todo cfg

        d.setStartingPose(startingPose);
        //
        //
        waitForStart();
        runAuto(d);

        while (opModeIsActive()) {
            d.update();
            d.telemetryDebug(FtcDashboard.getInstance().getTelemetry()); // disable for comp
        }
    }

    private void runAuto(Follower d) {
        PathChain test = d.pathBuilder()
                .addPath(bc(p(0, 0), p(4 * PLATE, 0)))
                .addPath(bc(p(4 * PLATE, 0), p(4 * PLATE, -4 * PLATE)))
                .addPath(bc(p(4 * PLATE, -4 * PLATE), p(0, -4 * PLATE)))
                .addPath(bc(p(0, -4 * PLATE), p(0, 0)))
                .build();

        cs.schedule(new SequentialCommandGroup(
                        new FollowPath(d, test),
                        new WaitCommand(3000),
                        new FollowPath(d, test)
                )
        );
    }

    private BezierCurve bc(Point... controlPoints) {
        return new BezierCurve(controlPoints);
    }

    private Point p(double x, double y) {
        return new Point(new Pose(x, y));
    }
}
