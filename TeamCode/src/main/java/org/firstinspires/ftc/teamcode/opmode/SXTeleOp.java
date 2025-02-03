package org.firstinspires.ftc.teamcode.opmode;



import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.action.Actions;

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
public class SXTeleOp extends OpMode  {

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

    private enum TeleOpState{
        TRANSFER_IN, TRANSFER_OUT, IN, OUT
    }

    private TeleOpState TeleState;

    @Override
    public void init() {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        TeleState = TeleOpState.TRANSFER_IN;

        lift = new Lift(hardwareMap);
        extend = new Extend(hardwareMap);
        outake = new Outake(hardwareMap);
        intake = new Intake(hardwareMap);

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        startSubsystems();


    }

    private void startSubsystems() {
        lift.start();
        extend.start();
        outake.start();
        intake.start();
    }

    private void switchToTransfer() {
        Actions.runBlocking(new SequentialAction(
                outake.openClaw,
                outake.toTransfer,
                intake.armToTransfer,
                extend.retractEx
        ));
    }

    private void switchToOuttake() {
        Actions.runBlocking(new SequentialAction(
                outake.closeClaw,
                new SleepAction(500),
                outake.toBucket,
                intake.armToTransfer,
                extend.retractEx
        ));
    }

    private void switchToIntake() {
        Actions.runBlocking(new SequentialAction(
                outake.openClaw,
                outake.toTransfer,
                extend.extendEx,
                new SleepAction(500),
                intake.armLower

        ));
    }

    @Override
    public void loop() {

        //Rising edge wannabe
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);


        //Intake Controls
        if (Intake.pivotState == Intake.PivotState.LOWER) {
            if (gamepad2.dpad_left) {
                intake.setSpin(Intake.IntakeState.STOP, false);
                telemetry.addLine("Intake Spin: Stopped");
            } else if (gamepad2.dpad_up) {
                intake.setSpin(Intake.IntakeState.REVERSE, false);
                telemetry.addLine("Intake Spin: Intaking");
            } else {
                intake.setSpin(Intake.IntakeState.FORWARD, false);
                telemetry.addLine("Intake Spin: Spinning Out");
            }
        } else {
            intake.setSpin(Intake.IntakeState.STOP, false);
        }

        //Tele State State Machine
        // transformers urr er uh ah eh eh
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            switch (TeleState) {
                case TRANSFER_IN:
                    switchToTransfer();

                    TeleState = TeleOpState.IN;
                    telemetry.addLine("Tele State: IN");
                    break;
                case IN:
                    switchToIntake();

                    TeleState = TeleOpState.TRANSFER_OUT;
                    telemetry.addLine("Tele State: TRANSFER OUT");
                    break;
                case TRANSFER_OUT:
                    switchToTransfer();

                    TeleState = TeleOpState.OUT;
                    telemetry.addLine("Tele State: OUT");
                    break;
                case OUT:
                    switchToOuttake();

                    TeleState = TeleOpState.TRANSFER_IN;
                    telemetry.addLine("Tele State: TRANSFER IN");
                    break;
            }
        }

        //TELEOP STATE FAILSAFE (GP2)
        if (currentGamepad2.y && !previousGamepad2.y) {
            switchToOuttake();

            telemetry.addLine("Tele State: TRANSFER IN");
            TeleState = TeleOpState.TRANSFER_IN;
        }

        if (currentGamepad2.b && !previousGamepad2.b) {
            switchToTransfer();

            TeleState = TeleOpState.IN;
        }

        if (currentGamepad2.x && !previousGamepad2.x) {
            switchToTransfer();

            TeleState = TeleOpState.OUT;
        }

        if (currentGamepad2.a && !previousGamepad2.a) {
            switchToIntake();

            TeleState = TeleOpState.TRANSFER_OUT;
        }

        if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            outake.switchClawState();
            telemetry.addLine("Claw State: " + outake.getClawState());
        }

        //Lift Controls (GP1)

        //Manual Lift
        double liftPower = gamepad1.right_trigger - gamepad1.left_trigger;
        if(Math.abs(liftPower) > 0.1) {
            lift.setManualPower(liftPower);
        }

        //Automatic Lift
        if(currentGamepad1.dpad_up && previousGamepad1.dpad_up) {
            Actions.runBlocking(lift.topBucket);
        }
        if(currentGamepad1.dpad_down && previousGamepad1.dpad_down) {
            Actions.runBlocking(lift.lowered);
        }
        if(currentGamepad1.dpad_left && previousGamepad1.dpad_left) {
            Actions.runBlocking(lift.l1Touch);
        }
        if(currentGamepad1.dpad_right && previousGamepad1.dpad_right) {
            Actions.runBlocking(lift.l2Touch);
        }


        //driveing nyoooommmm
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);


        follower.update();
        lift.update();
        telemetry.update();
    }


    @Override
    public void stop() {
        intake.lowerArm();
    }

}
