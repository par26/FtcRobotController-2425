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
import org.firstinspires.ftc.teamcode.common.action.SleepAction;
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
            if (gamepad2.dpad_left) {
                intake.setSpin(Intake.IntakeState.STOP, false);
            } else if (gamepad2.dpad_up) {
                intake.setSpin(Intake.IntakeState.REVERSE, false);
            } else {
                intake.setSpin(Intake.IntakeState.FORWARD, false);
            }
        } else {
            intake.setSpin(Intake.IntakeState.STOP, false);
        }

        //Alternate intake & bucket
        if ((currentGamepad1.left_bumper && !previousGamepad1.left_bumper) ||
                (currentGamepad1.right_bumper && !previousGamepad1.right_bumper)) {
            switch (Intake.pivotState) {
                case LOWER:
                    Actions.runBlocking(new SequentialAction(outake.toTransfer, intake.retractArm));
                    telemetry.addLine("Transfer State");
                case RETRACT:
                    if (currentGamepad1.left_bumper) {
                        Actions.runBlocking(new SequentialAction(intake.lowerArm, outake.toBucket));
                        telemetry.addLine("Intake/Outake: Bucket");
                    }

                    if (currentGamepad1.right_bumper) {
                        Actions.runBlocking(new SequentialAction(intake.lowerArm, outake.toSpecimen));
                        telemetry.addLine("Intake/Outake: Specimen");
                    }
            }
        }
        

        if (currentGamepad1.a && !previousGamepad1.a) {
            outake.switchClawState();
            telemetry.addLine("Claw State: " + outake.getClawState());
        }

        //TODO: implement wrist (in case of specimen hang)

        //Thought process to reversing controls:
        //Gamepad 1 will be doing near nothing when the depositing at bucket right, so
        // they can control how far it extends upwards.
        //While trying to get sample from sub, gamepad 2 can control that (will take synchronization but less hassle
        // for both parties overall)
        double liftPower = gamepad1.right_trigger - gamepad1.left_trigger;
        lift.setPower(-liftPower);

        //driveing nyoooommmm
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();



        telemetry.update();
    }


    @Override
    public void stop() {
        intake.lowerArm();
    }

}
