package org.firstinspires.ftc.teamcode.common.autonomous;

import static org.firstinspires.ftc.teamcode.common.autonomous.FieldConstants.*;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.follower.*;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.teamcode.common.action.Action;
import org.firstinspires.ftc.teamcode.common.action.ParallelAction;
import org.firstinspires.ftc.teamcode.common.action.SequentialAction;
import org.firstinspires.ftc.teamcode.common.action.SleepAction;
import org.firstinspires.ftc.teamcode.common.pedroPathing.FollowPathAction;
import org.firstinspires.ftc.teamcode.common.pedroPathing.HoldPointAction;
import org.firstinspires.ftc.teamcode.common.subsystem.Extend;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.subsystem.Lift;
import org.firstinspires.ftc.teamcode.common.subsystem.Outake;
import org.firstinspires.ftc.teamcode.common.utils.AutonConstants;


public class Auton {


    private RobotStart startLocation;

    public Follower follower;
    public Extend extend;
    public Intake intake;
    public Lift lift;
    public Outake outake;



    //public Path element1, score1, element2, score2, element3, score3;
    //public PathChain pushSamples, depositPreload, specimen1, specimen2, specimen3, grab1, grab2, grab3, park;
    //Paths & Pathchains
    public PathChain depositPreload, sample1, score1, sample2, score2, sample3, score3;
    public Path hangSpecimen1, grabSpecimen2, hangSpecimen2, grabSpecimen3, hangSpecimen3, grabSpecimen4, hangSpecimen4, grabSpecimen5, hangSpecimen5;
    public PathChain park;
    public PathChain pushSamples;
    //Poses
    //Both
    public Pose spawnPose, preloadPose, parkPose, parkControlPose, preloadControlPose;
    //Bucket
    //TODO: we gon need vision for bucket
    public Pose sample1Pose, sample2Pose, sample3Pose, scorePose;
    //Observation
    public Pose grabPose, specimen1Pose, specimen2Pose, specimen3Pose, specimen4Pose, specimen5Pose;
    //Push
    public Pose pushSeg1Pose, pushSeg1Control, pushSeg2Pose, pushSeg2Control, pushSeg3Pose, pushSeg3Control, pushSeg4Pose, pushSeg4Control;

    public Auton(HardwareMap hardwareMap, RobotStart startLocation, Follower follower) {

        this.follower = follower;
        this.startLocation = startLocation;
        this.extend = new Extend(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.lift = new Lift(hardwareMap);
        this.outake = new Outake(hardwareMap);

        startSubsystems();


        createPoses();

        follower.setMaxPower(0.8);


        buildPaths();
    }

    private void startSubsystems() {
        lift.start();
        extend.start();
        outake.start();
        intake.start();

        outake.toTransfer();
    }

    public void createPoses() {
        switch(startLocation) {
            case BUCKET:
                spawnPose = bucketStartPose;
                preloadPose = bucketPreloadPose;
                preloadControlPose = bucketPreloadControl;

                sample1Pose = bucketSampleTopPose;
                sample2Pose = bucketSampleMiddlePose;
                sample3Pose = bucketSampleBottomPose;

                scorePose = bucketScorePose;
                parkPose = bucketParkPose;
                parkControlPose = bucketParkControl;

                break;
            case OBSERVATION:
//                spawnPose = observationStartPose;
//                preloadPose = observationPreloadPose;
//                grabPose = observationGrabPose;
//                parkPose = observationParkPose;
//                specimen1Pose = specimen1FCPose;
//                specimen2Pose = specimen2FCPose;
//                specimen3Pose = specimen3FCPose;
//                specimen4Pose = specimen4FCPose;
//                specimen5Pose = specimen5FCPose;
//
//                pushSeg1Pose = pushSeg1FCPose;
//                pushSeg1Control = pushSeg1FCControl;
//                pushSeg2Pose = pushSeg2FCPose;
//                pushSeg2Control = pushSeg2FCControl;
//                pushSeg3Pose = pushSeg3FCPose;
//                pushSeg3Control = pushSeg3FCControl;
//                pushSeg4Pose = pushSeg4FCPose;
//                pushSeg4Control = pushSeg4FCControl;
                break;
            case TEST:
                spawnPose = new Pose(8.74, 55.22, Math.toRadians(270));
        }

        follower.setStartingPose(spawnPose);
    }

    public void buildPaths() {
        if (startLocation == RobotStart.BUCKET) {
            depositPreload = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(spawnPose), new Point(preloadControlPose), new Point(preloadPose)))
                    .setLinearHeadingInterpolation(spawnPose.getHeading(), preloadPose.getHeading())
                    .build();

            sample1 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(preloadPose),  new Point(sample1Pose)))
                    .setLinearHeadingInterpolation(preloadPose.getHeading(), sample1Pose.getHeading())
                    .build();

            score1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(sample1Pose), new Point(scorePose)))
                    .setLinearHeadingInterpolation(sample1Pose.getHeading(), scorePose.getHeading())
                    .build();

            sample2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(scorePose), new Point(sample2Pose)))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), sample2Pose.getHeading())
                    .build();

            score2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(sample2Pose), new Point(scorePose)))
                    .setLinearHeadingInterpolation(sample2Pose.getHeading(), scorePose.getHeading())
                    .build();

            sample3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(scorePose), new Point(sample3Pose)))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), sample3Pose.getHeading())
                    .build();

            sample3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(sample3Pose), new Point(scorePose)))
                    .setLinearHeadingInterpolation(sample3Pose.getHeading(), scorePose.getHeading())
                    .build();

            park = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                    .build();
        }

        if (startLocation == RobotStart.OBSERVATION) {
//            depositPreload = new Path(new BezierLine(new Point(spawnPose), new Point(preloadPose)));
//            depositPreload.setLinearHeadingInterpolation(spawnPose.getHeading(), preloadPose.getHeading());
//
//            pushSamples = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Point(preloadPose),
//                            new Point(pushSeg1Control),
//                            new Point(pushSeg1Pose)
//                    )).setLinearHeadingInterpolation(preloadPose.getHeading(), pushSeg1Pose.getHeading())
//                    .addPath(new BezierCurve(
//                            new Point(pushSeg1Pose),
//                            new Point(pushSeg2Control),
//                            new Point(pushSeg2Pose)
//                    )).setLinearHeadingInterpolation(pushSeg1Pose.getHeading(), pushSeg2Pose.getHeading())
//                    .addPath(new BezierCurve(
//                            new Point(pushSeg2Pose),
//                            new Point(pushSeg3Control),
//                            new Point(pushSeg3Pose)
//                    )).setLinearHeadingInterpolation(pushSeg2Pose.getHeading(), pushSeg3Pose.getHeading())
//                    .addPath(new BezierCurve(
//                            new Point(pushSeg3Pose),
//                            new Point(pushSeg4Control),
//                            new Point(pushSeg4Pose)
//                    )).setLinearHeadingInterpolation(pushSeg3Pose.getHeading(), pushSeg4Pose.getHeading())
//                    .build();
//
//            hangSpecimen1 = new Path(new BezierLine(new Point(grabPose), new Point(specimen1Pose)));
//            hangSpecimen1.setLinearHeadingInterpolation(grabPose.getHeading(), specimen1Pose.getHeading());
//
//            grabSpecimen2 = new Path(new BezierLine(new Point(specimen1Pose), new Point(grabPose)));
//            grabSpecimen2.setLinearHeadingInterpolation(specimen1Pose.getHeading(), grabPose.getHeading());
//
//            hangSpecimen2 = new Path(new BezierLine(new Point(grabPose), new Point(specimen2Pose)));
//            hangSpecimen2.setLinearHeadingInterpolation(grabPose.getHeading(), specimen2Pose.getHeading());
//
//            grabSpecimen3 = new Path(new BezierLine(new Point(specimen2Pose), new Point(grabPose)));
//            grabSpecimen3.setLinearHeadingInterpolation(specimen2Pose.getHeading(), grabPose.getHeading());
//
//            hangSpecimen3 = new Path(new BezierLine(new Point(grabPose), new Point(specimen3Pose)));
//            hangSpecimen3.setLinearHeadingInterpolation(grabPose.getHeading(), specimen3Pose.getHeading());
//
//            grabSpecimen4 = new Path(new BezierLine(new Point(specimen3Pose), new Point(grabPose)));
//            grabSpecimen4.setLinearHeadingInterpolation(specimen3Pose.getHeading(), grabPose.getHeading());
//
//            hangSpecimen4 = new Path(new BezierLine(new Point(grabPose), new Point(specimen4Pose)));
//            hangSpecimen4.setLinearHeadingInterpolation(grabPose.getHeading(), specimen4Pose.getHeading());
//
//            grabSpecimen5 = new Path(new BezierLine(new Point(specimen4Pose), new Point(grabPose)));
//            grabSpecimen5.setLinearHeadingInterpolation(specimen4Pose.getHeading(), grabPose.getHeading());
//
//            hangSpecimen5 = new Path(new BezierLine(new Point(grabPose), new Point(specimen5Pose)));
//            hangSpecimen5.setLinearHeadingInterpolation(grabPose.getHeading(), specimen5Pose.getHeading());
//
//            park = new Path(new BezierLine(new Point(scorePose), new Point(parkPose)));
//            park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
        }
    }

    public Action testPath() {
        PathChain tPath1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(8.74, 55.22), new Point(33.96, 41.11)))
                .setLinearHeadingInterpolation(Math.toRadians(270), 0)
                .build();
        PathChain tPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(33.96, 41.11), new Point(33.17, 26.62), new Point(9.14, 17.08)))
                .setLinearHeadingInterpolation(0, 0)
                .build();

      return new SequentialAction(
              new FollowPathAction(follower, tPath1),
              new ParallelAction(
                      new HoldPointAction(follower, new Pose(33.96, 41.11), 8000),
                      new SequentialAction(
                              intake.intakeIn,
                              new SleepAction(5000),
                              intake.intakeStop,
                              new SleepAction(2000),
                              extend.extendEx
                      )
              ),
              new FollowPathAction(follower, tPath2)
      );
    }

    public Action bucketSequence() {
        return new SequentialAction(
                depositPreload(),
                handlePickup(sample1),
                handleScore(score1),
                handlePickup(sample2),
                handleScore(score2),
                handlePickup(sample3),
                handleScore(score3),
                park()
        );
    }

    public Action handlePickup(PathChain pc) {
        Path p = pc.getPath(0);
        Point pt = p.getPoint(1);
        Pose pose = new Pose(pt.getX(), pt.getY());

        return new SequentialAction(
                new FollowPathAction(follower, pc),
                new ParallelAction(
                        new HoldPointAction(follower, pose, 4000),
                        new SequentialAction(
                                pickupFlat(),
                                inToTransfer()
                        )
                )
        );
    }

    public Action handleScore(PathChain pc) {
        Path p = pc.getPath(0);
        Point pt = p.getPoint(1);
        Pose pose = new Pose(pt.getX(), pt.getY());

        return new SequentialAction(
                new FollowPathAction(follower, pc),
                new ParallelAction(
                        new HoldPointAction(follower, pose, 4000),
                        new SequentialAction(
                                deposit(),
                                outToTransfer()
                        )
                )
        );
    }

    public Action park() {
        return new SequentialAction(
                new FollowPathAction(follower, park),
                new ParallelAction(
                        new HoldPointAction(follower, parkPose, 10000),
                        new SequentialAction(intake.intakeIn)
                )
        );
    }

    public Action depositPreload() {
        return new SequentialAction(
                new FollowPathAction(follower, depositPreload),
                new ParallelAction(
                        new HoldPointAction(follower, preloadPose, 10000),
                        deposit()
                )
        );
    }

    public Action deposit() {
        return new SequentialAction(
                outake.closeClaw,
                new SleepAction(AutonConstants.OUTAKE_CLAW),
                liftHighBucket(),
                new SleepAction(AutonConstants.LIFT_TOP_BUCKET),
                outake.toBucket,
                new SleepAction(AutonConstants.OUTAKE_ARM_SWITCH),
                outake.openClaw
        );
    }

    //TODO: adjust pickup accordingly
    public Action pickupFlat() {
        return new SequentialAction(
                intake.armLower,
                intake.intakeIn,
                extend.extendEx,
                new SleepAction(1000)
        );
    }

    //if vision
    public Action pickupSub() {
        return new SequentialAction(
                extend.extendEx,
                new SleepAction(AutonConstants.EXTEND_EX),
                intake.armLower,
                intake.intakeIn

        );
    }


    public Action outToTransfer() {
        return new SequentialAction(
                outake.toTransfer,
                outake.openClaw,
                new SleepAction(AutonConstants.OUTAKE_CLAW),
                liftLowered(),
                new SleepAction(AutonConstants.LIFT_LOW)

        );
    }

    public Action inToTransfer() {
        return new SequentialAction(
                intake.armToTransfer,
                intake.intakeStop,
                new SleepAction(AutonConstants.INTAKE_ARM_BUFFER),
                extend.retractEx,//hi bby girl
                new SleepAction(AutonConstants.RETRACT_EX)
        );
    }

    /*
    public Action pathOnly() {
        return new SequentialAction(
                new FollowPathAction(follower, depositPreload),
                new HoldPointAction(follower, bucketScorePose, 1000),
                new FollowPathAction(follower, sample1),
                new SleepAction(1000),
                new FollowPathAction(follower, score1),
                new SleepAction(1000),
                new FollowPathAction(follower, sample2),
                new SleepAction(1000),
                new FollowPathAction(follower, score2),
                new SleepAction(1000),
                new FollowPathAction(follower, sample3),
                new SleepAction(1000),
                new FollowPathAction(follower, score3)
        );
        
    }
    */
    public Action liftHighBucket() {
        return new SequentialAction(
                lift.topBucket,
                lift.waitSlide()
        );
    }

    public Action liftLowBucket() {
        return new SequentialAction(
                lift.lowBucket,
                lift.waitSlide()
        );
    }

    public Action liftL2Chamber() {
        return new SequentialAction(
                lift.l2Touch,
                lift.waitSlide()
        );
    }

    public Action liftL1Chamber() {
        return new SequentialAction(
                lift.l1Touch,
                lift.waitSlide()
        );
    }

    public Action liftLowered() {
        return new SequentialAction(
                lift.lowered,
                lift.waitSlide()
        );
    }

}

/*Ryan: 1430
* Bet:

* Sam: 580
* Bet:
*
*Pot: 140
 */

