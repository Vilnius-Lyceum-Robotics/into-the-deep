package org.firstinspires.ftc.teamcode.auto.pedroCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.VLRAuto;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

public class FollowPath extends CommandBase {
    private final Follower m_follower;
    private Path m_path = null;
    private PathChain m_pathChain = null;

    private Point pathEndPoint;

    public FollowPath(Follower follower, Path path) {
        m_follower = follower;
        m_path = path;
    }

    public FollowPath(Follower follower, PathChain pathChain) {
        m_follower = follower;
        m_pathChain = pathChain;
    }

    public FollowPath(Follower follower, double constantHeading, Point... points) {
        m_follower = follower;
        pathEndPoint = getLastPoint(points);

        if (points.length == 1){
            m_pathChain = follower.pathBuilder().addPath(new BezierLine(VLRAuto.getLastRobotPosition(), points[0]))
                    .setConstantHeadingInterpolation(Math.toRadians(constantHeading)).build();
        }
        else if (points.length >= 2){
            m_pathChain = follower.pathBuilder().addPath(new BezierCurve(getUpdatedPointArray(points)))
                    .setConstantHeadingInterpolation(Math.toRadians(constantHeading)).build();
        }
        else throw new RuntimeException("An end point needs to be added to the path");
    }


    public FollowPath(Follower follower, boolean reverseTangentialDirection, Point... points) {
        m_follower = follower;
        pathEndPoint = getLastPoint(points);

        if (points.length == 1){
            m_pathChain = follower.pathBuilder().addPath(new BezierLine(VLRAuto.getLastRobotPosition(), points[0]))
                    .setTangentHeadingInterpolation().setReversed(reverseTangentialDirection).build();
        }
        else if (points.length >= 2){
            m_pathChain = follower.pathBuilder().addPath(new BezierCurve(getUpdatedPointArray(points)))
                    .setTangentHeadingInterpolation().setReversed(reverseTangentialDirection).build();
        }
        else throw new RuntimeException("An end point needs to be added to the path");
    }


    public FollowPath(Follower follower, double startHeading, double endHeading, Point... points) {
        m_follower = follower;
        Point startPoint = VLRAuto.getLastRobotPosition();
        pathEndPoint = startPoint;

        if (points.length == 1){
            m_pathChain = follower.pathBuilder().addPath(new BezierLine(startPoint, points[0]))
                    .setLinearHeadingInterpolation(Math.toRadians(startHeading), Math.toRadians(endHeading)).build();
        }
        else if (points.length >= 2){
            m_pathChain = follower.pathBuilder().addPath(new BezierCurve(getUpdatedPointArray(points)))
                    .setLinearHeadingInterpolation(Math.toRadians(startHeading), Math.toRadians(endHeading)).build();
        }
        else{
            m_pathChain = follower.pathBuilder().addPath(new BezierLine(startPoint, startPoint))
                    .setLinearHeadingInterpolation(Math.toRadians(startHeading), Math.toRadians(endHeading)).build();
        }
    }


    private Point[] getUpdatedPointArray(Point... points){
        Point[] updatedArray = new Point[points.length + 1];
        updatedArray[0] = VLRAuto.getLastRobotPosition();
        System.arraycopy(points, 0, updatedArray, 1, points.length);

        return updatedArray;
    }

    private Point getLastPoint(Point... points){
        return points[points.length - 1];
    }


    @Override
    public void initialize() {
        if (m_path != null) {
            m_follower.followPath(m_path);
        } else {
            m_follower.followPath(m_pathChain);
        }
    }

    @Override
    public void execute() {
        m_follower.update();
    }

    @Override
    public void end(boolean isInterrupted){
        VLRAuto.setLastRobotPosition(pathEndPoint);
    }

    @Override
    public boolean isFinished() {
        return !m_follower.isBusy();
    }
}