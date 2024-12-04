package org.firstinspires.ftc.teamcode.opmode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.subsystem.Lift;


@Config
@TeleOp(name="PID_Test", group="z")
public class Slide_PID_Test extends OpMode {
    Lift lift;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target;



    private final double ticks_in_degrees = 537.7 / 360.0;

    @Override
    public void init() {
       lift = new Lift(hardwareMap);
    }

    @Override
    public void loop() {
      lift.updatePIDConstants(p, i, d, f);
      lift.update();
    }
}