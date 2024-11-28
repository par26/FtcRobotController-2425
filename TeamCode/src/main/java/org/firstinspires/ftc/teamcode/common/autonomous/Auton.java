package org.firstinspires.ftc.teamcode.common.autonomous;

import static org.firstinspires.ftc.teamcode.common.autonomous.FieldConstants.*;

import org.firstinspires.ftc.teamcode.common.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.common.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.Point;

public class Auton {


    private RobotStart startLocation;

    public Follower follower;

    //public Path element1, score1, element2, score2, element3, score3;
    //public PathChain pushSamples, depositPreload, specimen1, specimen2, specimen3, grab1, grab2, grab3, park;
    //Paths & Pathchains
    public Path depositPreload, sample1, score1, sample2, score2, sample3, score3;
    public PathChain pushSamples;
    //Poses
    //Both
    public Pose spawnPose, preloadPose, parkPose;
    //Bucket
    //TODO: we gon need vision for bucket
    public Pose sample1Control, sample1Pose, sample2Pose, sample3Pose, scorePose;
    //Observation
    public Pose grabPose, specimen1Pose, specimen2Pose, specimen3Pose, specimen4Pose, specimen5Pose;


    public Auton(RobotStart startLocation) {
        //TODO: add subsystems when they're done

        this.startLocation = startLocation;

        buildPaths();
        createPoses();


    }

    public void createPoses() {
        //TODO: Need to finish the actual poses,
        // bucket assigning should be done by now 11/22/24
        switch(startLocation) {
            case BLUE_BUCKET:
                spawnPose = blueBucketStartPose;
                preloadPose = blueBucketPreloadPose;
                sample1Control = blueSample1Control;
                sample1Pose = blueBucketSampleTopPose;
                sample2Pose = blueBucketSampleMiddlePose;
                sample3Pose = blueBucketSampleBottomPose;
                scorePose = blueBucketScorePose;
                parkPose = blueBucketParkPose;

                break;
            case BLUE_OBSERVATION:
                spawnPose = blueObservationStartPose;
                preloadPose = blueObservationPreloadPose;
                grabPose = blueObservationGrabPose;
                parkPose = blueObservationParkPose;
                specimen1Pose = blueSpecimen1Pose;
                specimen2Pose = blueSpecimen2Pose;
                specimen3Pose = blueSpecimen3Pose;
                specimen4Pose = blueSpecimen4Pose;
                specimen5Pose = blueSpecimen5Pose;

                break;
            case RED_BUCKET:
                spawnPose = redBucketStartPose;
                preloadPose = redBucketPreloadPose;
                sample1Control = redSample1Control;
                sample1Pose = redBucketSampleTopPose;
                sample2Pose = redBucketSampleMiddlePose;
                sample3Pose = redBucketSampleBottomPose;
                scorePose = redBucketScorePose;
                parkPose = redBucketParkPose;

                break;
            case RED_OBSERVATION:
                spawnPose = redObservationStartPose;
                preloadPose = redObservationPreloadPose;
                grabPose = redObservationGrabPose;
                parkPose = redObservationParkPose;
                specimen1Pose = redSpecimen1Pose;
                specimen2Pose = redSpecimen2Pose;
                specimen3Pose = redSpecimen3Pose;
                specimen4Pose = redSpecimen4Pose;
                specimen5Pose = redSpecimen5Pose;

                break;
        }

        follower.setStartingPose(spawnPose);
    }

    public void buildPaths() {
        if (startLocation == RobotStart.RED_BUCKET || startLocation == RobotStart.BLUE_BUCKET) {
            depositPreload = new Path(new BezierLine(new Point(spawnPose), new Point(preloadPose)));
            depositPreload.setLinearHeadingInterpolation(spawnPose.getHeading(), preloadPose.getHeading());

            sample1 = new Path(new BezierCurve(new Point(preloadPose), new Point(sample1Control), new Point(sample1Pose)));
            sample1.setLinearHeadingInterpolation(preloadPose.getHeading(), sample1Pose.getHeading());

            score1 = new Path(new BezierLine(new Point(sample1Pose), new Point(scorePose)));
            score1.setLinearHeadingInterpolation(sample1Pose.getHeading(), scorePose.getHeading());

            sample2 = new Path(new BezierLine(new Point(scorePose), new Point(sample2Pose)));
            sample2.setLinearHeadingInterpolation(scorePose.getHeading(), sample2Pose.getHeading());

            score2 = new Path(new BezierLine(new Point(sample2Pose), new Point(scorePose)));
            score2.setLinearHeadingInterpolation(sample2Pose.getHeading(), scorePose.getHeading());

            sample3 = new Path(new BezierLine(new Point(scorePose), new Point(sample3Pose)));
            sample3.setLinearHeadingInterpolation(scorePose.getHeading(), sample3Pose.getHeading());

            score3 = new Path(new BezierLine(new Point(sample3Pose), new Point(scorePose)));
            score3.setLinearHeadingInterpolation(sample3Pose.getHeading(), scorePose.getHeading());


        }

        if (startLocation == RobotStart.RED_OBSERVATION || startLocation == RobotStart.BLUE_OBSERVATION) {
            depositPreload = new Path(new BezierLine(new Point(spawnPose), new Point(preloadPose)));
            depositPreload.setLinearHeadingInterpolation(spawnPose.getHeading(), preloadPose.getHeading());

            pushSamples = follower.pathBuilder()
                    .addPath(new BezierCurve())
                    .build();
        }
    }
}
