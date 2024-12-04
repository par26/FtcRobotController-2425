//package org.firstinspires.ftc.teamcode.common;
//
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.common.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.common.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.common.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.common.pedroPathing.util.Timer;
//import org.firstinspires.ftc.teamcode.common.subsystem.Extend;
//import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
//import org.firstinspires.ftc.teamcode.common.subsystem.Lift;
//import org.firstinspires.ftc.teamcode.common.subsystem.Outake;
//
//public class Auto {
//
//    private RobotStart startLocation;
//
//    public Outake claw;
//    public Lift lift;
//    public Extend extend;
//    public Intake intake;
//
//
//    public Follower follower;
//    public Telemetry telemetry;
//
//    public boolean actionBusy, liftPIDF = true;
//    public double liftManual = 0;
//
//    public Timer transferTimer = new Timer(), bucketTimer = new Timer(), chamberTimer = new Timer(), intakeTimer = new Timer(), parkTimer = new Timer(), specimenTimer = new Timer(), chamberTimer2 = new Timer();
//    public int transferState = -1, bucketState = -1, chamberState = -1, intakeState = -1, parkState = -1, specimenState = -1;
//
//    public Path element1, score1, element2, score2, element3, score3;
//    public PathChain pushSamples, preload, specimen1, specimen2, specimen3, specimen4, grab1, grab2, grab3, grab4, park;
//    public Pose startPose, preloadPose, sample1Pose, sample1ControlPose, sample2Pose, sample2ControlPose, sample3Pose, sample3ControlPose, sampleScorePose, parkControlPose, parkPose, grab1Pose, specimen1Pose, grab2Pose, specimen2Pose, grab3Pose, specimen3Pose, grab4Pose, specimen4Pose, specimenSetPose;
//
//    public Auto(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, boolean isBlue, boolean isBucket) {
//        claw = new Outake(hardwareMap);
//        lift = new Lift(hardwareMap, telemetry);
//        extend = new Extend(hardwareMap, telemetry);
//        intake = new Intake(hardwareMap);
//
//
//        this.follower = follower;
//        this.telemetry = telemetry;
//
//        startLocation = isBlue ? (isBucket ? RobotStart.BLUE_BUCKET : RobotStart.BLUE_OBSERVATION) : (isBucket ? RobotStart.RED_BUCKET : RobotStart.RED_OBSERVATION);
//
//        createPoses();
//        buildPaths();
//
//        follower.setStartingPose(startPose);
//
//        init();
//    }
//
//    public void init() {
//        claw.init();
//        claw.openClaw();
//        lift.init();
//        extend.toZero();
//        intake.init();
//        telemetryUpdate();
//    }
//
//    public void start() {
//        lift.start();
//        extend.start();
//        intake.start();
//        claw.closeClaw();
//    }
//
//    public void update() {
//
//
//        lift.update();
//
//
//        telemetryUpdate();
//    }
//
//    public void createPoses() { //Able to be cut
//        switch (startLocation) {
//            case BLUE_BUCKET:
//                startPose = blueBucketStartPose;
//                preloadPose = blueBucketPreloadPose;
//                sample1ControlPose = blueBucketLeftSampleControlPose;
//                sample1Pose = blueBucketLeftSamplePose;
//                sample2ControlPose = blueBucketMidSampleControlPose;
//                sample2Pose = blueBucketMidSamplePose;
//                sample3ControlPose = blueBucketRightSampleControlPose;
//                sample3Pose = blueBucketRightSamplePose;
//                sampleScorePose = blueBucketScorePose;
//                parkControlPose = blueBucketParkControlPose;
//                parkPose = blueBucketParkPose;
//                break;
//
//            case BLUE_OBSERVATION:
//                startPose = blueObservationStartPose;
//                preloadPose = blueObservationPreloadPose;
//                specimenSetPose = blueObservationSpecimenSetPose;
//                grab1Pose = blueObservationSpecimenPickupPose;
//                grab2Pose = blueObservationSpecimenPickup2Pose;
//                grab3Pose = blueObservationSpecimenPickup3Pose;
//                grab4Pose = blueObservationSpecimenPickup4Pose;
//                specimen1Pose = blueObservationSpecimen1Pose;
//                specimen2Pose = blueObservationSpecimen2Pose;
//                specimen3Pose = blueObservationSpecimen3Pose;
//                specimen4Pose = blueObservationSpecimen4Pose;
//
//
//                parkPose = blueObservationParkPose;
//                break;
//
//            case RED_BUCKET:
//                startPose = redBucketStartPose;
//                //parkPose = redBucketPark;
//                break;
//
//            case RED_OBSERVATION:
//                startPose = redObservationStartPose;
//                //parkPose = redObservationPark;
//                break;
//        }
//
//        follower.setStartingPose(startPose);
//    }
//
//    public void buildPaths() {
//        if ((startLocation == RobotStart.BLUE_BUCKET) || (startLocation == RobotStart.RED_BUCKET)) {
//            preload = follower.pathBuilder()
//                    .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
//                    .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
//                    .build();
//
//            element1 = new Path(new BezierCurve(new Point(preloadPose), new Point(sample1ControlPose), new Point(sample1Pose)));
//            element1.setLinearHeadingInterpolation(preloadPose.getHeading(), sample1Pose.getHeading());
//
//            score1 = new Path(new BezierLine(new Point(sample1Pose), new Point(sampleScorePose)));
//            score1.setLinearHeadingInterpolation(sample1Pose.getHeading(), sampleScorePose.getHeading());
//
//            element2 = new Path(new BezierCurve(new Point(sampleScorePose), new Point(sample2ControlPose), new Point(sample2Pose)));
//            element2.setLinearHeadingInterpolation(sampleScorePose.getHeading(), sample2Pose.getHeading());
//
//            score2 = new Path(new BezierLine(new Point(sample2Pose), new Point(sampleScorePose)));
//            score2.setLinearHeadingInterpolation(sample2Pose.getHeading(), sampleScorePose.getHeading());
//
//            element3 = new Path(new BezierCurve(new Point(sampleScorePose), new Point(sample3ControlPose), new Point(sample3Pose)));
//            element3.setLinearHeadingInterpolation(sampleScorePose.getHeading(), sample3Pose.getHeading());
//
//            score3 = new Path(new BezierLine(new Point(sample3Pose), new Point(sampleScorePose)));
//            score3.setLinearHeadingInterpolation(sample3Pose.getHeading(), sampleScorePose.getHeading());
//
//            park = follower.pathBuilder()
//                    .addPath(new BezierCurve(new Point(sampleScorePose), new Point(parkControlPose), new Point(parkPose)))
//                    .setLinearHeadingInterpolation(sampleScorePose.getHeading(), parkPose.getHeading())
//                    .build();
//        }
//
//        if (startLocation == RobotStart.BLUE_OBSERVATION || startLocation == RobotStart.RED_OBSERVATION) {
//            preload = follower.pathBuilder()
//                    .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
//                    .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
//                    .setZeroPowerAccelerationMultiplier(4)
//                    .build();
//
//            pushSamples = follower.pathBuilder()
//                    .addPath(new BezierCurve(new Point(preloadPose), new Point(15, 36, Point.CARTESIAN), new Point(61, 36.25, Point.CARTESIAN), new Point(59, 26.000, Point.CARTESIAN)))
//                    .setLinearHeadingInterpolation(preloadPose.getHeading(), Math.toRadians(180))
//                    .addPath(new BezierLine(new Point(59.000, 26.000, Point.CARTESIAN), new Point(28, 26.000, Point.CARTESIAN)))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .addPath(new BezierCurve(new Point(28, 26.000, Point.CARTESIAN), new Point(52.000, 30.000, Point.CARTESIAN), new Point(58.000, 16.000, Point.CARTESIAN)))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .addPath(new BezierLine(new Point(58.000, 16.000, Point.CARTESIAN), new Point(28, 16.000, Point.CARTESIAN)))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .addPath(new BezierCurve(new Point(28, 16.000, Point.CARTESIAN), new Point(56.000, 16.000, Point.CARTESIAN), new Point(56.000, 10, Point.CARTESIAN)))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .addPath(new BezierLine(new Point(56.000, 10, Point.CARTESIAN), new Point(28, 10, Point.CARTESIAN)))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .addPath(new BezierLine(new Point(28, 10, Point.CARTESIAN), new Point(specimenSetPose)))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), specimenSetPose.getHeading())
//                    //.setZeroPowerAccelerationMultiplier(0.5)
//                    .build();
//
//            grab1 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Point(specimenSetPose), new Point(grab1Pose)))
//                    .setLinearHeadingInterpolation(specimenSetPose.getHeading(), grab1Pose.getHeading())
//                    .setZeroPowerAccelerationMultiplier(1)
//                    .build();
//
//            specimen1 = follower.pathBuilder()
//                    .addPath(new BezierCurve(new Point(grab1Pose), new Point(specimen1Pose.getX() - 10, specimen1Pose.getY(), Point.CARTESIAN), new Point(specimen1Pose)))
//                    .setLinearHeadingInterpolation(grab1Pose.getHeading(), specimen1Pose.getHeading())
//                    .setZeroPowerAccelerationMultiplier(1)
//                    .build();
//
//            grab2 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Point(specimen1Pose), new Point(grab2Pose)))
//                    .setLinearHeadingInterpolation(specimen1Pose.getHeading(), grab2Pose.getHeading())
//                    .setZeroPowerAccelerationMultiplier(1)
//                    .build();
//
//            specimen2 = follower.pathBuilder()
//                    .addPath(new BezierCurve(new Point(grab2Pose), new Point(specimen2Pose.getX() - 10, specimen2Pose.getY(), Point.CARTESIAN), new Point(specimen2Pose)))
//                    .setLinearHeadingInterpolation(grab2Pose.getHeading(), specimen2Pose.getHeading())
//                    .setZeroPowerAccelerationMultiplier(1)
//                    .build();
//
//            grab3 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Point(specimen2Pose), new Point(grab3Pose)))
//                    .setLinearHeadingInterpolation(specimen2Pose.getHeading(), grab3Pose.getHeading())
//                    .setZeroPowerAccelerationMultiplier(1)
//                    .build();
//
//            specimen3 = follower.pathBuilder()
//                    .addPath(new BezierCurve(new Point(grab3Pose), new Point(specimen3Pose.getX() - 10, specimen3Pose.getY(), Point.CARTESIAN), new Point(specimen3Pose)))
//                    .setLinearHeadingInterpolation(grab3Pose.getHeading(), specimen3Pose.getHeading())
//                    .setZeroPowerAccelerationMultiplier(1)
//                    .build();
//
//            grab4 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Point(specimen3Pose), new Point(grab4Pose)))
//                    .setLinearHeadingInterpolation(specimen3Pose.getHeading(), grab4Pose.getHeading())
//                    .setZeroPowerAccelerationMultiplier(1)
//                    .build();
//
//            specimen4 = follower.pathBuilder()
//                    .addPath(new BezierCurve(new Point(grab4Pose), new Point(specimen4Pose.getX() - 10, specimen3Pose.getY(), Point.CARTESIAN), new Point(specimen3Pose)))
//                    .setLinearHeadingInterpolation(grab4Pose.getHeading(), specimen4Pose.getHeading())
//                    .setZeroPowerAccelerationMultiplier(1)
//                    .build();
//
//            park = follower.pathBuilder()
//                    .addPath(new BezierLine(new Point(specimen4Pose), new Point(parkPose)))
//                    .setLinearHeadingInterpolation(specimen4Pose.getHeading(), parkPose.getHeading())
//                    .setZeroPowerAccelerationMultiplier(3)
//                    .build();
//
//        }
//    }
//
//
//    public void telemetryUpdate() {
//        telemetry.addData("X: ", follower.getPose().getX());
//        telemetry.addData("Y: ", follower.getPose().getY());
//        telemetry.addData("Heading: ", follower.getPose().getHeading());
//        telemetry.addData("Action Busy?: ", actionBusy);
//        //telemetry.addData("Lift Pos", lift.getPos());
//        //telemetry.addData("Extend Pos", extend.leftExtend.getPosition());
//        //telemetry.addData("Extend Limit", extend.extendLimit);
//        telemetry.addData("Claw Grab State", claw.grabState);
//        telemetry.addData("Claw Pivot State", claw.pivotState);
//        //   telemetry.addData("Intake Spin State", intakeSpinState);
//        //   telemetry.addData("Intake Pivot State", intakePivotState);
//        telemetry.addData("arm State", arm.state);
//        telemetry.update();
//    }
//}