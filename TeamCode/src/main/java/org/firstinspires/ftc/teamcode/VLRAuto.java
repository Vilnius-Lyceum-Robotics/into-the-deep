package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;

@Photon
@Autonomous(name = "VLRAuto")
public class VLRAuto extends VLRLinearOpMode implements VLRAutoConfiguration {
    CommandScheduler cs;

    Follower d;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(Chassis.class, Pinpoint.class);
        VLRSubsystem.initializeAll(hardwareMap);

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
        // Example blue auto where 3 of the side samples are pushed to the human player
        Pose startPose = StartPose.BLUE_RIGHT.pose;
        d.setStartingPose(startPose);
        BezierCurve toFirst = bc(
                p(startPose),
                p(-2 * PLATE, startPose.getY()),
                p(-PLATE, -1.5 * PLATE),
                p(0, -2 * PLATE + 0.5)
        );

        BezierCurve pushFirst = bc(
                p(0, -2 * PLATE + 0.5),
                p(-2.5 * PLATE + ROBOT_L / 2 + 12, -2 * PLATE + 0.5)
        );

        Path pushFirstPath = new Path(pushFirst);
        pushFirstPath.setConstantHeadingInterpolation(0);

        BezierCurve toSecond = bc(
                p(-2.5 * PLATE + ROBOT_L / 2 + 12, -2 * PLATE + 0.5),
                p(PLATE, -2 * PLATE + 0.5),
                p(0, -2.5 * PLATE + 0.5)
        );

        Path toSecondPath = new Path(toSecond);
        toSecondPath.setConstantHeadingInterpolation(0);

        BezierCurve pushSecond = bc(
                p(0, -2.5 * PLATE + 0.5),
                p(-2.5 * PLATE + ROBOT_L / 2 + 12, -2.5 * PLATE + 0.5)
        );

        Path pushSecondPath = new Path(pushSecond);
        pushSecondPath.setConstantHeadingInterpolation(0);

        BezierCurve toThird = bc(
                p(-2.5 * PLATE + ROBOT_L / 2 + 12, -2.5 * PLATE + 0.5),
                p(PLATE, -2.5 * PLATE + 0.5),
                p(0, -3 * PLATE + ROBOT_W / 2 + 0.5)
        );

        Path toThirdPath = new Path(toThird);
        toThirdPath.setConstantHeadingInterpolation(0);

        BezierCurve pushThird = bc(
                p(0, -3 * PLATE + ROBOT_W / 2 + 0.5),
                p(-2.5 * PLATE + ROBOT_L / 2 + 12, -3 * PLATE + ROBOT_W / 2 + 0.5)
        );

        Path pushThirdPath = new Path(pushThird);
        pushThirdPath.setConstantHeadingInterpolation(0);

        cs.schedule(new SequentialCommandGroup(
                        new FollowPath(d, new Path(toFirst)),
                        new FollowPath(d, pushFirstPath),
                        new FollowPath(d, new Path(toSecond)),
                        new FollowPath(d, pushSecondPath),
                        new FollowPath(d, new Path(toThird)),
                        new FollowPath(d, pushThirdPath)
                )
        );

    }

    private BezierCurve bc(Point... controlPoints) {
        return new BezierCurve(controlPoints);
    }

    private Point p(double x, double y) {
        return new Point(new Pose(x, y));
    }

    private Point p(Pose pose) {
        return new Point(pose);
    }
}
