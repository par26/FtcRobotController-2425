package org.firstinspires.ftc.teamcode.common.autonomous;

import org.firstinspires.ftc.teamcode.common.pedroPathing.localization.Pose;

public class FieldConstants {

    /* Robot Dimensions
     * Length: 16.562
     * Width: 14.242
     * Robot deposits sample in bucket from back
     * Robot intakes sample from front
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
    public static final Pose blueBucketPreloadPose = new Pose(30.25, 78.375, Math.toRadians(180));
    public static final Pose blueObservationPreloadPose = new Pose(30.25, 60.625, Math.toRadians(180));
    public static final Pose redBucketPreloadPose = new Pose(144-blueBucketPreloadPose.getX(), blueBucketPreloadPose.getY(), 0);
    public static final Pose redObservationPreloadPose = new Pose(144-blueObservationPreloadPose.getX(), blueObservationPreloadPose.getY(), 0);

    //Bucket Score Poses TODO: adjust accordingly
    public static final Pose blueBucketScore = new Pose(17.6, 125.3, Math.toRadians(315));
    public static final Pose redBucketScore = new Pose(144- blueBucketScore.getX(), 144- blueBucketScore.getY(), Math.toRadians(135));

    //Observation Grab Poses
    public static final Pose blueObservationGrab = new Pose(12.3, 34, Math.toRadians(270));
    public static final Pose redObservationGrab = new Pose(144- blueObservationGrab.getX(), 144- blueObservationGrab.getY(), Math.toRadians(90));

    //BLue Specimen Exact Poses
    //TODO: If changing specific positions, only changing blueSampleTop necessary
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


    //Parking Poses
    //TODO: will need adjusting, not minmaxed
    public static final Pose blueObservationParkPose = new Pose(60, 47, 0);
    public static final Pose redObservationParkPose = new Pose(144- blueObservationParkPose.getX(), 144- blueObservationParkPose.getY(), 0);
    public static final Pose blueBucketParkPose = new Pose(blueObservationParkPose.getX(), 144- blueObservationParkPose.getY(), 0);
    public static final Pose redBucketParkPose = new Pose(144- blueObservationParkPose.getX(), blueObservationParkPose.getY(), 0);


}
