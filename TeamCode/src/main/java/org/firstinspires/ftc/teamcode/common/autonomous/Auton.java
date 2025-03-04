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

        follower.setMaxPower(0.65);


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
                spawnPose = new Pose(8.74, 55.22, 0);
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
                .setLinearHeadingInterpolation(0, 0)
                .build();
        PathChain tPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(33.96, 41.11), new Point(33.17, 26.62), new Point(9.14, 17.08)))
                .setLinearHeadingInterpolation(0, Math.toRadians(180))
                .build();

      return new SequentialAction(
              new FollowPathAction(follower, tPath1),
              outake.closeClaw,
              new HoldPointAction(follower, new Pose(33.96, 41.11), 4000),
              outake.openClaw,
              new SleepAction(2000),
              outake.closeClaw,
              new FollowPathAction(follower, tPath2),
              outake.openClaw
      );
    }


    public Action testBucket() {
        return new SequentialAction(
                new FollowPathAction(follower, depositPreload),
                depositSampleHigh(),
                resetBot(),
                extendBot(),
                new FollowPathAction(follower, sample1),
                pickUpSample(),
                new FollowPathAction(follower, score1),
                 depositSampleHigh(),
                resetBot(),
                extendBot(),
                new FollowPathAction(follower, sample2),
                pickUpSample(),
                new FollowPathAction(follower, score2),
                depositSampleHigh(),
                resetBot(),
                extendBot(),
                new FollowPathAction(follower, sample3),
                pickUpSample(),
                new FollowPathAction(follower, score3),
                depositSampleHigh(),
                resetBot()
        );

    }

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

    public Action DepositPreload1() {
        return new SequentialAction(
                new ParallelAction(
                        new FollowPathAction(follower, depositPreload)
                        //new HoldPointAction(follower, bucketScorePose, 4000)
                ),
                new ParallelAction(

                intake.armLower,
                outake.toBucket,
                intake.intakeIn
                )
        );
    }

    public Action extendBot() {
        return new SequentialAction(
                extend.extendEx,
                intake.armLower
        );
    }

    public Action depositPreload() {
        return new ParallelAction(
                new HoldPointAction(follower, bucketScorePose, 7),
                new SequentialAction(
                //TODO: Add subsystem actions
                new ParallelAction(

                        intake.armLower,
                        outake.toBucket
                        //intake.intakeIn
                ),


                new SleepAction(300),
                new ParallelAction(
                        intake.intakeOut,
                        intake.armToTransfer,
                        outake.toTransfer
                ),
                new SleepAction(300),
                new FollowPathAction(follower, depositPreload),
                depositSampleHigh(),


                new SleepAction(300),
                resetBot()
        ));
    }

    public Action handleBucketChain() {
        //TODO: use parallel action when auton is substituted with extend working and follow path and extend at the same time
        return new SequentialAction(
                new FollowPathAction(follower, sample1),
                extendBot(),
                new SleepAction(1000),
                new ParallelAction(
                        new HoldPointAction(follower, sample1Pose, 5000),
                        pickUpSample()
                )
               ,
                new FollowPathAction(follower, score1),
                depositSampleHigh(),
                resetBot(),
                extendBot(),
                new SleepAction(1000),
                new FollowPathAction(follower, sample2),
                new ParallelAction(
                        new HoldPointAction(follower, sample2Pose, 5000),
                        pickUpSample()
                ),
                new FollowPathAction(follower, score2),
                depositSampleHigh(),
                resetBot(),
                new SleepAction(1000),
                new FollowPathAction(follower, sample3),
                new ParallelAction(
                        new HoldPointAction(follower, sample3Pose, 5000),
                        pickUpSample()
                ),
                new FollowPathAction(follower, score3),
                depositSampleHigh(),
                resetBot(),
                new SleepAction(1000)
        );
    }

//    public Action visionSample() {
//
//    }

    public Action hangSpecimen() {
        return new SequentialAction(
                //TODO: Implement outake actions

        );
    }

    //Bucket Specific
    public Action pickUpSample() {
        return new SequentialAction(
                intake.intakeIn,
                new SleepAction(1000),
                intake.armToTransfer,
                new SleepAction(700),
                extend.retractEx,
                intake.intakeStop,
                new SleepAction(300),
                outake.closeClaw
        );
    }

    //Bucket Specific
    public Action depositSampleHigh() {

        return new ParallelAction(
                new HoldPointAction(follower, bucketScorePose, 3000),
                new SequentialAction(
                outake.closeClaw,
                liftHighBucket(),
                new SleepAction(150),
                outake.toBucket,
                new SleepAction(400),
                outake.openClaw,


                new SleepAction(300)
        ));

    }

    public Action resetBot() {
        return new SequentialAction(
                outake.toTransfer,
                outake.openClaw,
               liftLowered(),
                intake.armLower
        );
    }

    public Action liftHighBucket() {
        return new SequentialAction(
                lift.topBucket,
                lift.waitSlide()
        );
    }

    public Action liftLowBucket() {
        return new SequentialAction(

                new ParallelAction(
                        new HoldPointAction(follower, sample2Pose, 5000),
                        pickUpSample()
                ),


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

    //Dual Action
    public Action park() {
        return new SequentialAction(
                new FollowPathAction(follower, park)
                //impl liftl1 chamber or l2 chamber
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

