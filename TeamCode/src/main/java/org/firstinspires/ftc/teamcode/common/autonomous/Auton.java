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
    public Path hangSpecimen1, grabSpecimen2, hangSpecimen2, grabSpecimen3, hangSpecimen3, grabSpecimen4, hangSpecimen4, grabSpecimen5, hangSpecimen5;
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
                    .addPath(new BezierCurve(
                            new Point(38.250, 60.625, Point.CARTESIAN),
                            new Point(29.758, 42.437, Point.CARTESIAN),
                            new Point(59.645, 31.439, Point.CARTESIAN)
                    )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(190))
                    .addPath(new BezierCurve(
                            new Point(59.645, 31.439, Point.CARTESIAN),
                            new Point(68.442, 22.900, Point.CARTESIAN),
                            new Point(16.173, 20.571, Point.CARTESIAN)
                    )).setLinearHeadingInterpolation(Math.toRadians(190), Math.toRadians(180))
                    .addPath(new BezierCurve(
                            new Point(16.173, 20.571, Point.CARTESIAN),
                            new Point(130.415, 27.558, Point.CARTESIAN),
                            new Point(20.184, 15.137, Point.CARTESIAN)
                    )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(195))
                    .addPath(new BezierCurve(
                            new Point(20.184, 15.137, Point.CARTESIAN),
                            new Point(104.669, 11.127, Point.CARTESIAN),
                            new Point(18.631, 3.752, Point.CARTESIAN)
                    )).setLinearHeadingInterpolation(Math.toRadians(195), Math.toRadians(180))
                    .build();

            hangSpecimen1 = new Path(new BezierLine(new Point(grabPose), new Point(specimen1Pose)));
            hangSpecimen1.setLinearHeadingInterpolation(grabPose.getHeading(), specimen1Pose.getHeading());

            grabSpecimen2 = new Path(new BezierLine(new Point(specimen1Pose), new Point(grabPose)));
            grabSpecimen2.setLinearHeadingInterpolation(specimen1Pose.getHeading(), grabPose.getHeading());

            hangSpecimen2 = new Path(new BezierLine(new Point(grabPose), new Point(specimen2Pose)));
            hangSpecimen2.setLinearHeadingInterpolation(grabPose.getHeading(), specimen2Pose.getHeading());

            grabSpecimen3 = new Path(new BezierLine(new Point(specimen2Pose), new Point(grabPose)));
            grabSpecimen3.setLinearHeadingInterpolation(specimen2Pose.getHeading(), grabPose.getHeading());

            hangSpecimen3 = new Path(new BezierLine(new Point(grabPose), new Point(specimen3Pose)));
            hangSpecimen3.setLinearHeadingInterpolation(grabPose.getHeading(), specimen3Pose.getHeading());

            grabSpecimen4 = new Path(new BezierLine(new Point(specimen3Pose), new Point(grabPose)));
            grabSpecimen4.setLinearHeadingInterpolation(specimen3Pose.getHeading(), grabPose.getHeading());

            hangSpecimen4 = new Path(new BezierLine(new Point(grabPose), new Point(specimen4Pose)));
            hangSpecimen4.setLinearHeadingInterpolation(grabPose.getHeading(), specimen4Pose.getHeading());

            grabSpecimen5 = new Path(new BezierLine(new Point(specimen4Pose), new Point(grabPose)));
            grabSpecimen5.setLinearHeadingInterpolation(specimen4Pose.getHeading(), grabPose.getHeading());

            hangSpecimen5 = new Path(new BezierLine(new Point(grabPose), new Point(specimen5Pose)));
            hangSpecimen5.setLinearHeadingInterpolation(grabPose.getHeading(), specimen5Pose.getHeading());
        }
    }
}
