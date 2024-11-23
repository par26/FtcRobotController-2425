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
    //Both
    public Pose spawnPose, preloadPose, parkPose;
    //Bucket
    //TODO: we gon need vision for bucket
    public Pose sample1Pose, sample2Pose, sample3Pose, scorePose;
    //Observation
    public Pose grabPose, specimen1Pose, specimen2Pose, specimen3Pose, specimen4Pose;


    public Auton(RobotStart startLocation) {
        //TODO: add subsystems when they're done

        this.startLocation = startLocation;

        buildPaths();
        createPoses();


    }

    public void createPoses() {
        //TODO: Need to finish the actual poses, bucket assigning
        // should be done by now 11/22/24
        switch(startLocation) {
            case BLUE_BUCKET:
                spawnPose = blueBucketStartPose;
                preloadPose = blueBucketPreloadPose;
                sample1Pose = blueBucketSampleTopPose;
                sample2Pose = blueBucketSampleMiddlePose;
                sample3Pose = blueBucketSampleBottomPose;
                scorePose = blueBucketScore;
                parkPose = blueBucketParkPose;

                break;
            case BLUE_OBSERVATION:
                spawnPose = blueObservationStartPose;
                preloadPose = blueObservationPreloadPose;
                grabPose = blueObservationGrab;
                parkPose = blueObservationParkPose;

                break;
            case RED_BUCKET:
                spawnPose = redBucketStartPose;
                preloadPose = redBucketPreloadPose;
                sample1Pose = redBucketSampleTopPose;
                sample2Pose = redBucketSampleMiddlePose;
                sample3Pose = redBucketSampleBottomPose;
                scorePose = redBucketScore;
                parkPose = redBucketParkPose;

                break;
            case RED_OBSERVATION:
                spawnPose = redObservationStartPose;
                preloadPose = redObservationPreloadPose;
                grabPose = redObservationGrab;
                parkPose = redObservationParkPose;

                break;
        }

        follower.setStartingPose(spawnPose);
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
