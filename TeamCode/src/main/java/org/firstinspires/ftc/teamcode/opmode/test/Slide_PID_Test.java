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



    Lift lift;
    public static double p = 0.04, i = 0, d = 0.000001, f = 0.01;
    public static int target = 200;



    private final double ticks_in_degrees = 537.7 / 360.0;

    Gamepad lastGamepad1 = new Gamepad();


    private boolean usePIDF = false;

    @Override
    public void init() {
        lift = new Lift(hardwareMap);

        lift.init();
        lift.start();
    }

    @Override
    public void loop() {

        lift.updatePIDConstants(p, i, d, f);


        // This is a rising-edge detector that runs if and only if "a" was pressed this loop.
        if (gamepad1.a && !lastGamepad1.a) {
            usePIDF = true;
        }


        if (gamepad1.left_trigger > 0) {
            lift.setManualPower(-gamepad1.left_trigger);
            usePIDF = false;
        } else if (gamepad1.right_trigger > 0) {
            lift.setManualPower(gamepad1.right_trigger);

            // If we get any sort of manual input, turn PIDF off.
            usePIDF = false;
        } else if (usePIDF) {
            lift.setTarget(target);

        }


        telemetry.addData("lift pos", lift.getCurrentPos());
        telemetry.addData("lift target", target);
        telemetry.update();
        lift.update();
    }
}