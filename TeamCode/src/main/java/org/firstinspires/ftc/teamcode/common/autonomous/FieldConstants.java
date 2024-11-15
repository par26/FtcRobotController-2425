package org.firstinspires.ftc.teamcode.common.autonomous;

import org.firstinspires.ftc.teamcode.common.pedroPathing.localization.Pose;

public class FieldConstants {

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


    //BLue Specimen Exact Poses
    //TODO: If changing specific positions, only changing blueSampleTop necessary
    public static final Pose blueSampleTopPose      = new Pose(45.8, 23, 0);
    public static final Pose blueSampleMiddlePose   = new Pose(45.8, 12.65, 0);
    public static final Pose blueSampleBottomPose   = new Pose(45.8, 2.25, 0);

    //Red Specimen Exact Poses
    public static final Pose redSampleTopPose       = new Pose(144-blueSampleTopPose.getX(), 144-blueSampleTopPose.getY(), 0);
    public static final Pose redSampleMiddlePose    = new Pose(144-blueSampleMiddlePose.getX(), 144-blueSampleTopPose.getY(), 0);
    public static final Pose redSampleBottomPose    = new Pose(144-blueSampleBottomPose.getX(), 144-blueSampleBottomPose.getY(), 0);

    //Blue Observation
    public static final Pose blueBucketSampleTopPose      = new Pose(blueSampleTopPose.getX(), 144-blueSampleTopPose.getY(), 0);
    public static final Pose blueBucketSampleMiddlePose      = new Pose(blueSampleTopPose.getX(), 144-blueSampleTopPose.getY(), 0);
    public static final Pose blueBucketSampleBottomPose      = new Pose(blueSampleTopPose.getX(), 144-blueSampleTopPose.getY(), 0);

    //Red Observation
    public static final Pose redBucketSampleTopPose      = new Pose(144-blueSampleTopPose.getX(), blueSampleTopPose.getY(), 0);
    public static final Pose redBucketSampleMiddlePose      = new Pose(144-blueSampleTopPose.getX(), blueSampleTopPose.getY(), 0);
    public static final Pose redBucketSampleBottomPose      = new Pose(144-blueSampleTopPose.getX(), blueSampleTopPose.getY(), 0);


    //Parking Poses
    //TODO: will need adjusting, not minmaxed
    public static final Pose blueSampleParkingPose  = new Pose(60, 47, 0);
    public static final Pose redSampleParkingPose   = new Pose(144- blueSampleParkingPose.getX(), 144- blueSampleParkingPose.getY(), 0);
    public static final Pose blueBucketParkingPose  = new Pose(blueSampleParkingPose.getX(), 144- blueSampleParkingPose.getY(), 0);
    public static final Pose redBucketParkingPose   = new Pose(144- blueSampleParkingPose.getX(), blueSampleParkingPose.getY(), 0);


}
