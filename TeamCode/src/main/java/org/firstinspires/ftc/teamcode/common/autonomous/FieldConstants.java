package org.firstinspires.ftc.teamcode.common.autonomous;


import com.pedropathing.localization.Pose;

public class FieldConstants {

    /* Robot Dimensions
     * 12/30/24
     *  ------
     * |      |
     * |robot |
     * |      | Width
     * |      |
     *  ------
     *  Height
     *
     * Height: 14.5
     * Width: 16.5
     * Robot deposits sample in bucket from back
     * Robot intakes sample from front
     *
     * Misc Values?:
     * Specimen gap: 3.5
     */

    //TODO: Change bucket accordingly for meet
        //for the record, only changed  bucket so far for upgraded but no extension bot

    public enum
    RobotStart {
        BUCKET,
        OBSERVATION,
    }

    public static final Pose bucketStartPose = new Pose(7.116, 87.59, Math.toRadians(270));
    public static final Pose observationStartPose = new Pose(9.5, 65.25, Math.toRadians(180));

    // Preload Poses
    // TODO: Adjust accordingly how far robot can reach
    public static final Pose bucketPreloadPose = new Pose(14.5, 129, Math.toRadians(315));
    public static final Pose observationPreloadPose = new Pose(38.25, 60.625, Math.toRadians(180));

    public static final Pose bucketPreloadControl = new Pose(15.66, 93.02);

    //Bucket Score Poses TODO: adjust accordingly
    public static final Pose bucketScorePose = new Pose(14.5, 129, Math.toRadians(315));

    //Observation Grab Poses
    public static final Pose observationGrabPose = new Pose(12.3, 33, Math.toRadians(270));

    //Observation Deposit Poses
    //Change only Specimen1 values
    public static final Pose specimen1FCPose = new Pose(37, 62, Math.toRadians(180));
    public static final Pose specimen2FCPose = new Pose(specimen1FCPose.getX(), specimen1FCPose.getY()+3.5, Math.toRadians(180));
    public static final Pose specimen3FCPose = new Pose(specimen1FCPose.getX(), specimen2FCPose.getY()+3.5, Math.toRadians(180));
    public static final Pose specimen4FCPose = new Pose(specimen1FCPose.getX(), specimen3FCPose.getY()+3.5, Math.toRadians(180));
    public static final Pose specimen5FCPose = new Pose(specimen1FCPose.getX(), specimen4FCPose.getY()+3.5, Math.toRadians(180));

    //Bucket Sample Action Poses

//    public static final Pose bucketSampleTopPose = new Pose(35.4, 132.7, 0);
//    public static final Pose bucketSampleMiddlePose = new Pose(35.4, 132, 0);
//    public static final Pose bucketSampleBottomPose = new Pose(39.3, 132.48,  Math.toRadians(60));

    // To Replace w/ Extend
    public static final Pose bucketSampleTopPose = new Pose(25, 125, Math.toRadians(345));
    public static final Pose bucketSampleMiddlePose = new Pose(25, 129, Math.toRadians(8));
    public static final Pose bucketSampleBottomPose = new Pose(25, 134,  Math.toRadians(20));

    //Push Points (im gonna cray cray)
    public static final Pose pushSeg1FCPose = new Pose(59.64,31.44, Math.toRadians(190));
    public static final Pose pushSeg1FCControl = new Pose(29.758, 42.437);
    public static final Pose pushSeg2FCPose = new Pose(15.78, 23.03, Math.toRadians(180));
    public static final Pose pushSeg2FCControl = new Pose(68.18, 25.23);
    public static final Pose pushSeg3FCPose = new Pose(19.8, 14.49, Math.toRadians(195));
    public static final Pose pushSeg3FCControl = new Pose(129.38, 27.04);
    public static final Pose pushSeg4FCPose = new Pose(20.05, 6.08, Math.toRadians(180));
    public static final Pose pushSeg4FCControl = new Pose(107.51, 13.33);

    //Parking Poses
    //TODO: will need adjusting, not minmaxed
    public static final Pose observationParkPose = new Pose(60, 47, 0);

    public static final Pose bucketParkPose = new Pose(59, 96, Math.toRadians(90));
    public static final Pose bucketParkControl = new Pose(70.38, 106.87);


}
