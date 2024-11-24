package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class TestClaw extends OpMode {


    boolean pb, cb = false;

    //rising edge for the a button
    boolean pa, ca = false;

    //rising edge for the x button
    boolean px, cx = false;

    //rising edge for the y button
    boolean py, cy = false;

    //rising edge for left trigger
    boolean plt, clt = false;

    //rising edge for right trigger
    boolean prt, crt = false;

    //rising edge for the left bumper
    boolean plb, clb = false;

    //rising edge for right bumper
    boolean prb, crb = false;

    //rising edge for back button
    boolean pbb, cbb = false;

    //rising edge for start button
    boolean psb, csb = false;

    Servo leftClaw;
    Servo rightClaw;

    public static double leftOpen = 0.0;

    public static double rightOpen = 0.0;

    public static double leftClose = .2;

    public static double rightClose = .2;

    //determines if the left is reverse or not
    public static int leftReverse = 0;

    //determines if the right is reverse or not
    public static int rigthReverse = 0;
    @Override
    public void init() {
        leftClaw = hardwareMap.get(Servo.class, "servo");

        //rightClaw = hardwareMap.get(Servo.class, "rightServo");

        if(leftReverse == 1) {
            leftClaw.setDirection(Servo.Direction.REVERSE);
        } else {
            leftClaw.setDirection(Servo.Direction.FORWARD);
        }


        leftClaw.setPosition(0);


    }





    @Override
    public void loop() {

        //Open left
        px = cx;
        cx = gamepad1.x;
        if (cx && !px) {
            leftClaw.setPosition(leftOpen);
            double lpos = leftClaw.getPosition();
            telemetry.addData("Position Left", lpos);
            telemetry.update();

        }

        //Close left
        pa = ca;
        ca = gamepad1.a;
        if (ca && !pa) {
            leftClaw.setPosition(leftClose);
            double pos = leftClaw.getPosition();
            telemetry.addData("Position Left", pos);
            telemetry.update();

        }



        /* px = cx;
        cx = gamepad1.a;
        if(gamepad1.a) {
            servo3.setPosition(1);
            double pos = servo3.getPosition();
            telemetry.addData("position", pos);
            telemetry.update();

        }

        double position = servo3.getPosition();
        telemetry.addData("position", position);
        telemetry.update(); */


    }
}