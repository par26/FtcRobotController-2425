package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.action.Actions;
import org.firstinspires.ftc.teamcode.common.action.SequentialAction;
import org.firstinspires.ftc.teamcode.common.subsystem.Extend;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.subsystem.Outake;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestPivoting extends OpMode {

//    public static double leftServoAngle;
//    public static double rightServoAngle;
//
//    public static int leftDirection = 0;
//    public static int rightDirection = 0;

    Intake intake;
    Outake outake;
    Extend extend;

    ElapsedTime timer;
    ServoImplEx leftOuttake;
    ServoImplEx rightOuttake;

    ServoImplEx leftIntake, rightIntake;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {

        intake = new Intake(hardwareMap);
        outake = new Outake(hardwareMap);
        extend = new Extend(hardwareMap);

        intake.start();
        outake.start();
        extend.start();
        new SequentialAction(intake.armLower, outake.toTransfer);
    }


    @Override
    public void loop() {
        //Open left

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.a && !previousGamepad1.a) {
            Actions.runBlocking(outake.toTransfer);
            telemetry.addLine("Outake: Transfer");

        }

        if (currentGamepad1.b && !previousGamepad1.b) {
            Actions.runBlocking(outake.toBucket);
            telemetry.addLine("Outake: Bucket");

        }

        if (currentGamepad1.x && !previousGamepad1.x) {
            Actions.runBlocking(intake.armLower);
            telemetry.addLine("Intake: Lower Arm");

        }

        if (currentGamepad1.y && !previousGamepad1.y) {
            Actions.runBlocking(intake.armToTransfer);
            telemetry.addLine("Intake: Retracted Arm");
        }

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            outake.switchClawState();
        }

        telemetry.update();
    }
}