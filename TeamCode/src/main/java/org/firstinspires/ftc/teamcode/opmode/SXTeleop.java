package org.firstinspires.ftc.teamcode.opmode;



import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.action.Actions;

import org.firstinspires.ftc.teamcode.common.action.ParallelAction;
import org.firstinspires.ftc.teamcode.common.action.SequentialAction;
import org.firstinspires.ftc.teamcode.common.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.common.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Extend;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.subsystem.Lift;
import org.firstinspires.ftc.teamcode.common.subsystem.Outake;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class SXTeleop extends OpMode  {

    private Follower follower;

    private final double MAX_RETRACT_ANGLE = 200.0/360.0;

    public static double idle_power = 0.0;

    double clawOpen = 0;
    double clawClose = 0.4;

    Lift lift;
    Extend extend;
    Outake outake;
    Intake intake;

    private final Pose startPose = new Pose(0,0,0);

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    Intake.IntakeState teleopState;

    double intakePower = 0;

    public static double rightOpen = 0.5;
    public static double leftClose = .2;
    public static double rightClose = .2;

    @Override
    public void init() {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        teleopState = Intake.IntakeState.STOP;

        lift = new Lift(hardwareMap);
        extend = new Extend(hardwareMap);
        outake = new Outake(hardwareMap);
        intake = new Intake(hardwareMap);


    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        initSubsystems();
        startSubsystems();
    }


    private void initSubsystems() {
        lift.init();
        extend.init();
        outake.init();
        intake.init();
    }

    private void startSubsystems() {
        lift.start();
        extend.start();
        outake.start();
        intake.start();

        outake.toTransfer();
    }

    @Override
    public void loop() {

        //Rising edge wannabe
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        //Intake Controls
        if (Intake.pivotState == Intake.PivotState.LOWER) {
            if (gamepad1.dpad_left || Intake.pivotState == Intake.PivotState.RETRACT) {
                intake.setSpin(Intake.IntakeState.STOP, false);
            } else if (gamepad1.dpad_up) {
                intake.setSpin(Intake.IntakeState.FORWARD, false);
            } else {
                intake.setSpin(Intake.IntakeState.REVERSE, false);
            }
        }

        //Transfer to Bucket
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            if (Intake.pivotState == Intake.PivotState.LOWER) {
                Actions.runBlocking(new SequentialAction( new ParallelAction(intake.retractArm,
                outake.toTransfer, outake.closeClaw)));
            } else {
                Actions.runBlocking(new SequentialAction( new ParallelAction(intake.lowerArm,
                        outake.toBucket, outake.openClaw)));
            }
        }

        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            if (Intake.pivotState == Intake.PivotState.LOWER) {
                intake.retractArm();
                outake.toTransfer();
                outake.closeClaw();
            } else {
                intake.lowerArm();
                outake.toSpeicmen();
                outake.openClaw();
            }
        }


        if(currentGamepad2.x && previousGamepad2.x) {
            outake.toTransfer();
        }


        if(currentGamepad2.y && previousGamepad2.y) {
            Actions.runBlocking(new ParallelAction(outake.toBucket, intake.lowerArm));
        }

        if (currentGamepad2.a && !previousGamepad2.a) {
            telemetry.addLine("Claw button pressed");
            outake.switchClawState();
        }



        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            //extend.setManualPower(0.7);
            intake.retractArm();
        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            //extend.setManualPower(-0.7);
            intake.lowerArm();
        }



        double liftPower = gamepad2.right_trigger - gamepad2.left_trigger;

        lift.setPower(-liftPower);


        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();



        // telemetry.addData("Claw close value", clawClosed);
        telemetry.update();

    }


    @Override
    public void stop() {
        intake.lowerArm();
    }

}
