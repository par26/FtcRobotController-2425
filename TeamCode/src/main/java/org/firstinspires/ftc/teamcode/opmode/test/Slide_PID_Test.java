package org.firstinspires.ftc.teamcode.opmode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.subsystem.Lift;


@Config
@TeleOp(name="PID_Test", group="z")
public class Slide_PID_Test extends OpMode {


    DcMotorEx rightMotor, leftMotor;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 200;



    private final double ticks_in_degrees = 537.7 / 360.0;

    Gamepad lastGamepad1 = new Gamepad();


    private boolean usePIDF = false;

    @Override
    public void init() {

        rightMotor = hardwareMap.get(DcMotorEx.class, "rightExtend");
        //rightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftExtend");

        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        rightMotor = hardwareMap.get(DcMotorEx.class, "rightExtend");
        //rightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftExtend");

        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // This is a rising-edge detector that runs if and only if "a" was pressed this loop.
        if (gamepad1.a && !lastGamepad1.a) {
            usePIDF = true;
        }


        if (gamepad1.left_trigger > 0) {
            rightMotor.setPower(-gamepad1.left_trigger);

            // If we get any sort of manual input, turn PIDF off.
            usePIDF = false;
        } else if (gamepad1.right_trigger > 0) {
            leftMotor.setPower(gamepad1.right_trigger);

            // If we get any sort of manual input, turn PIDF off.
            usePIDF = false;
        }
    }
}