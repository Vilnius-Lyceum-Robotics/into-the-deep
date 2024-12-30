package org.firstinspires.ftc.teamcode.auto.commandFactory;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.commands.GrabBucketSample;
import org.firstinspires.ftc.teamcode.helpers.commands.ScoreBucketSample;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class NetCommandFactory extends CommandFactory {
    private int scoreHeading;
    private int sample1Heading;
    private int sample2Heading;
    private int sample3Heading;

    private Point startingPoint;
    private Point toScore;
    private Point toSamples1And2;
    private Point toSample3;

    public NetCommandFactory(boolean isBlueTeam){
        initializeHeadings();
        initializePointsForBlueTeam();
        if(!isBlueTeam){
            Point[] allPoints = {
                    startingPoint, toScore, toSamples1And2, toSample3
            };
            mirrorPointsToRedTeam(allPoints);
        }
    }

    private void initializeHeadings(){
        scoreHeading = -50;
        sample1Heading = -13;
        sample2Heading = 5;
        sample3Heading = 28;
    }
    @Override
    public void initializePointsForBlueTeam(){
        startingPoint = new Point(10, 111.5);
        toScore = new Point(22, 123);
        toSamples1And2 = new Point(22, 125);
        toSample3 = new Point(25, 125);
    }
    @Override
    public Point getStartingPoint() {
        return startingPoint;
    }
    @Override
    public SequentialCommandGroup getCommands(){
        return new SequentialCommandGroup(
                new SetClawTwist(ClawConfiguration.TargetTwist.NORMAL),
                new FollowPath(0, scoreHeading, toScore),
                new ScoreBucketSample(),

                new FollowPath(scoreHeading, sample1Heading, toSamples1And2),
                new GrabBucketSample(),

                new FollowPath(sample1Heading, scoreHeading, toScore),
                new ScoreBucketSample(),

                new FollowPath(scoreHeading, sample2Heading, toSamples1And2),
                new GrabBucketSample(),

                new FollowPath(sample2Heading, scoreHeading, toScore),
                new ScoreBucketSample(),

                new FollowPath(scoreHeading, sample3Heading, toSample3),
                new GrabBucketSample(),

                new FollowPath(sample3Heading, scoreHeading, toScore),
                new ScoreBucketSample()
        );
    }
}
