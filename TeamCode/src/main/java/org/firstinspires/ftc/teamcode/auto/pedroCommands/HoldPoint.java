package org.firstinspires.ftc.teamcode.auto.pedroCommands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

public class HoldPoint extends CommandBase {
    private final Follower m_follower;
    private final BezierPoint m_point;
    private final double m_heading;

    public HoldPoint(Follower follower, BezierPoint point, double heading) {
        m_follower = follower;
        m_point = point;
        m_heading = heading;
    }

    public HoldPoint (Follower follower, Pose2d pose) {
        m_follower = follower;
        m_point = new BezierPoint(new Point(new Pose(pose.position.x, pose.position.y)));
        m_heading = pose.heading.toDouble();
    }

    @Override
    public void initialize() {
        m_follower.holdPoint(m_point, m_heading);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
