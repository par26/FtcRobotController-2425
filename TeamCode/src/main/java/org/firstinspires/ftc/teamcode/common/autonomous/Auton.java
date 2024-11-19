package org.firstinspires.ftc.teamcode.common.autonomous;

import static org.firstinspires.ftc.teamcode.common.autonomous.FieldConstants.*;

import org.firstinspires.ftc.teamcode.common.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.common.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.PathChain;

public class Auton {


    private RobotStart startLocation;

    public Follower follower;

    public Path element1, score1, element2, score2, element3, score3;
    public PathChain pushSamples, depositPreload, specimen1, specimen2, specimen3, grab1, grab2, grab3, park;
    public Pose startPose, preloadPose, sample1Pose, sample1ControlPose, sample2Pose, sample2ControlPose, sample3Pose, sample3ControlPose, sampleScorePose, parkControlPose, parkPose, grab1Pose, specimen1Pose, grab2Pose, specimen2Pose, grab3Pose, specimen3Pose, specimenSetPose;


    public Auton(RobotStart startLocation) {
        //TODO: add subsystems when they're done

        this.startLocation = startLocation;

        buildPaths();
        createPoses();


    }

    public void createPoses() {
        //TODO: For time coming fater 11/18/24, ID the poses we need and make it
        // (don't follow indubitables they too indubitable [they doing their own thing])
        switch(startLocation) {
            case BLUE_BUCKET:
                startPose = blueBucketStartPose;
                preloadPose = blueBucketPreloadPose;
                sample1Pose = blueBucketSampleTopPose;





                break;
            case RED_BUCKET:
                startPose = redBucketStartPose;

                break;
            case BLUE_OBSERVATION:
                startPose = blueObservationStartPose;

                break;
            case RED_OBSERVATION:
                startPose = redObservationStartPose;

                break;
        }

        follower.setStartingPose(startPose);
    }

    public void buildPaths() {
        if (startLocation == RobotStart.RED_BUCKET || startLocation == RobotStart.BLUE_BUCKET) {
            depositPreload = follower.pathBuilder()
                    .addPath(new BezierLine(new Point )
        }

        if (startLocation == RobotStart.RED_OBSERVATION || startLocation == RobotStart.BLUE_OBSERVATION) {

        }
    }
}
