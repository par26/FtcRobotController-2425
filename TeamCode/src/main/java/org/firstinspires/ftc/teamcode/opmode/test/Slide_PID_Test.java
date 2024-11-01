package org.firstinspires.ftc.teamcode.opmode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp(name="PID_Test", group="z")
public class Slide_PID_Test extends OpMode {
    private PIDController liftPID;

    private DcMotor motor1;
    private DcMotor motor2;
    private int pos;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target;



    private final double ticks_in_degrees = 537.7 / 360.0;

    @Override
    public void init() {
        liftPID = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor1 = hardwareMap.get(DcMotor.class, "leftSlide");
        motor2 = hardwareMap.get(DcMotor.class, "rightSlide");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        liftPID.setPID(p,i,d);
        int pos = motor1.getCurrentPosition();
        double pid = liftPID.calculate(pos, target);
        double ff = f;

        double power = pid + ff;

        motor1.setPower(power);
        telemetry.addData("lift pos", pos);
        telemetry.addData("lift target", target);
    }
}