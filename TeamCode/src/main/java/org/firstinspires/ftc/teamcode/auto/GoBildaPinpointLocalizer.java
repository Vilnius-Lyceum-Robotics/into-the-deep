package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.util.NanoTimer;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pose2D;

public class GoBildaPinpointLocalizer extends Localizer {
    private Pinpoint pinpoint;
    private Pose startPose;
    private Pose displacementPose;
    private Pose currentVelocity;
    private NanoTimer timer;
    private long deltaTimeNano;
    private final double InchToMmRatio = 25.4;


    public GoBildaPinpointLocalizer(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose());
    }

    public GoBildaPinpointLocalizer(HardwareMap hardwareMap, Pose setStartPose) {
        VLRSubsystem.requireSubsystems(Pinpoint.class);
        VLRSubsystem.initializeAll(hardwareMap);

        pinpoint = VLRSubsystem.getInstance(Pinpoint.class);
        pinpoint.setOffsets(fromMmToIn(75),fromMmToIn(-5));
        setStartPose(setStartPose);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        displacementPose = new Pose();
        currentVelocity = new Pose();
    }

    @Override
    public Pose getPose() {
        Pose p = MathFunctions.addPoses(startPose, displacementPose);
        System.out.println(String.format("pose %f %f %f", p.getX(), p.getY(), p.getHeading()));
        return MathFunctions.addPoses(startPose, displacementPose);
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity.copy();
    }

    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        displacementPose = MathFunctions.subtractPoses(setPose, startPose);
        pinpoint.resetIMU();
    }

    @Override
    public void update() {
        pinpoint.update();
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        Pose2D currentPosition = pinpoint.getPose();
        Pose2D velocity = pinpoint.getVelocity();

        // Convert Pinpoint's Pose2D to Pedro's Pose
        displacementPose = new Pose(
                fromMmToIn(currentPosition.getX()),
                fromMmToIn(currentPosition.getY()),
                currentPosition.getHeading()
        );

        // Update velocity
        currentVelocity = new Pose(
                velocity.getX(),
                velocity.getY(),
                velocity.getHeading()
        );
    }

    @Override
    public double getTotalHeading() {
        // THIS INVERSION IS HOLDING TOGETHER THE WHOLE UNIVERSE
        // DO NOT REMOVE
        return pinpoint.getPose().getHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return 1;
    }

    @Override
    public double getLateralMultiplier() {
        return 1;
    }

    @Override
    public double getTurningMultiplier() {
        return 1;
    }

    @Override
    public void resetIMU() {
        pinpoint.resetIMU();
    }

    public double fromMmToIn(double mm) {
        return mm / InchToMmRatio;
    }

    public double fromInToMm(double in) {
        return in * InchToMmRatio;
    }

}