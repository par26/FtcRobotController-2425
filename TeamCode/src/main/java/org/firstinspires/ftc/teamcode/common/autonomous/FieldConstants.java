package org.firstinspires.ftc.teamcode.common.autonomous;

import org.firstinspires.ftc.teamcode.common.pedroPathing.localization.Pose;

public class FieldConstants {

    /* Robot Dimensions
     * Length: 16.562
     * Width: 14.242
     * Robot deposits sample in bucket from back
     * Robot intakes sample from front
     *
     * Misc Values?:
     * Specimen gap: 3.5
     */

    public enum
    RobotStart {
        BLUE_BUCKET,
        BLUE_OBSERVATION,
        RED_BUCKET,
        RED_OBSERVATION
    }

    public static final Pose blueBucketStartPose = new Pose(9.5, 84, Math.toRadians(180));
    public static final Pose blueObservationStartPose = new Pose(9.5, 65.25, Math.toRadians(180));
    public static final Pose redBucketStartPose = new Pose(144-blueBucketStartPose.getX(), blueBucketStartPose.getY(), 0);
    public static final Pose redObservationStartPose = new Pose(144-blueObservationStartPose.getX(), blueObservationStartPose.getY(), 0);

    // Preload Poses
    // TODO: Adjust accordingly how far robot can reach
    public static final Pose blueBucketPreloadPose = new Pose(38.25, 78.375, Math.toRadians(180));
    public static final Pose blueObservationPreloadPose = new Pose(38.25, 60.625, Math.toRadians(180));
    public static final Pose redBucketPreloadPose = new Pose(144-blueBucketPreloadPose.getX(), blueBucketPreloadPose.getY(), 0);
    public static final Pose redObservationPreloadPose = new Pose(144-blueObservationPreloadPose.getX(), blueObservationPreloadPose.getY(), 0);

    //Bucket Score Poses TODO: adjust accordingly
    public static final Pose blueBucketScorePose = new Pose(17.6, 125.3, Math.toRadians(315));
    public static final Pose redBucketScorePose = new Pose(144- blueBucketScorePose.getX(), 144- blueBucketScorePose.getY(), Math.toRadians(135));

    //Observation Grab Poses
    public static final Pose blueObservationGrabPose = new Pose(12.3, 34, Math.toRadians(270));
    public static final Pose redObservationGrabPose = new Pose(144- blueObservationGrabPose.getX(), 144- blueObservationGrabPose.getY(), Math.toRadians(90));

    //Observation Deposit Poses
    //Change only blueSpecimen1 values
    public static final Pose blueSpecimen1Pose = new Pose(37, 62, Math.toRadians(180));
    public static final Pose blueSpecimen2Pose = new Pose(blueSpecimen1Pose.getX(), blueSpecimen1Pose.getY()+3.5, Math.toRadians(180));
    public static final Pose blueSpecimen3Pose = new Pose(blueSpecimen1Pose.getX(), blueSpecimen2Pose.getY()+3.5, Math.toRadians(180));
    public static final Pose blueSpecimen4Pose = new Pose(blueSpecimen1Pose.getX(), blueSpecimen3Pose.getY()+3.5, Math.toRadians(180));
    public static final Pose blueSpecimen5Pose = new Pose(blueSpecimen1Pose.getX(),blueSpecimen4Pose.getY()+3.5, Math.toRadians(180));

    public static final Pose redSpecimen1Pose = new Pose(144-blueSpecimen1Pose.getX(), 62, Math.toRadians(180));
    public static final Pose redSpecimen2Pose = new Pose(redSpecimen1Pose.getX(), redSpecimen1Pose.getY()+3.5, 0);
    public static final Pose redSpecimen3Pose = new Pose(redSpecimen2Pose.getX(), redSpecimen2Pose.getY()+3.5, 0);
    public static final Pose redSpecimen4Pose = new Pose(redSpecimen3Pose.getX(), redSpecimen3Pose.getY()+3.5, 0);
    public static final Pose redSpecimen5Pose = new Pose(redSpecimen4Pose.getX(),redSpecimen4Pose.getY()+3.5, 0);

    //Sample Action Poses
    public static final Pose blueSample1Control = new Pose(30.5, 87);
    public static final Pose redSample1Control = new Pose(144- blueSample1Control.getX(), 144- blueSample1Control.getY());

    public static final Pose blueBucketSampleTopPose = new Pose(35, 121.5, 0);
    public static final Pose blueBucketSampleMiddlePose = new Pose(35, 132, 0);
    public static final Pose blueBucketSampleBottomPose = new Pose(40, 132.7,  Math.toRadians(60));

    public static final Pose redBucketSampleTopPose = new Pose(144- blueBucketSampleTopPose.getX(), blueBucketSampleTopPose.getY(), Math.toRadians(180));
    public static final Pose redBucketSampleMiddlePose = new Pose(144- blueBucketSampleMiddlePose.getX(), 144- blueBucketSampleMiddlePose.getY(), Math.toRadians(180));
    public static final Pose redBucketSampleBottomPose = new Pose(144- blueBucketSampleBottomPose.getX(), 144- blueBucketSampleBottomPose.getY(), Math.toRadians(240));

    /*
    //Sample Exact Poses
    // If changing specific positions, only changing blueSampleTop necessary
    public static final Pose blueObservationSampleTopPose      = new Pose(45.8, 23, 0);
    public static final Pose blueObservationSampleMiddlePose = new Pose(45.8, 12.65, 0);
    public static final Pose blueObservationSampleBottomPose = new Pose(45.8, 2.25, 0);

    //Red Specimen Exact Poses
    public static final Pose redSampleTopPose       = new Pose(144-blueObservationSampleTopPose.getX(), 144-blueObservationSampleTopPose.getY(), 0);
    public static final Pose redSampleMiddlePose    = new Pose(144- blueObservationSampleMiddlePose.getX(), 144-blueObservationSampleTopPose.getY(), 0);
    public static final Pose redSampleBottomPose    = new Pose(144- blueObservationSampleBottomPose.getX(), 144- blueObservationSampleBottomPose.getY(), 0);

    //Blue Observation
    public static final Pose blueBucketSampleTopPose      = new Pose(blueObservationSampleTopPose.getX(), 144-blueObservationSampleTopPose.getY(), 0);
    public static final Pose blueBucketSampleMiddlePose      = new Pose(blueObservationSampleTopPose.getX(), 144-blueObservationSampleTopPose.getY(), 0);
    public static final Pose blueBucketSampleBottomPose      = new Pose(blueObservationSampleTopPose.getX(), 144-blueObservationSampleTopPose.getY(), 0);

    //Red Observation
    public static final Pose redBucketSampleTopPose      = new Pose(144-blueObservationSampleTopPose.getX(), blueObservationSampleTopPose.getY(), 0);
    public static final Pose redBucketSampleMiddlePose      = new Pose(144-blueObservationSampleTopPose.getX(), blueObservationSampleTopPose.getY(), 0);
    public static final Pose redBucketSampleBottomPose      = new Pose(144-blueObservationSampleTopPose.getX(), blueObservationSampleTopPose.getY(), 0);
    */

    //Parking Poses
    //TODO: will need adjusting, not minmaxed
    public static final Pose blueObservationParkPose = new Pose(60, 47, 0);
    public static final Pose redObservationParkPose = new Pose(144- blueObservationParkPose.getX(), 144- blueObservationParkPose.getY(), 0);
    public static final Pose blueBucketParkPose = new Pose(blueObservationParkPose.getX(), 144- blueObservationParkPose.getY(), 0);
    public static final Pose redBucketParkPose = new Pose(144- blueObservationParkPose.getX(), blueObservationParkPose.getY(), 0);


}