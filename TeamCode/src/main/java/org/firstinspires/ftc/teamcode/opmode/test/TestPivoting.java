package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.Teleop;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestPivoting extends Teleop {

    public static double leftServoAngle;
    public static double rightServoAngle;

    public static int leftDirection = 0;
    public static int rightDirection = 0;

    ElapsedTime timer;
    ServoImplEx leftServo;
    ServoImplEx rightServo;


    boolean cx, px, ca, pa = false;

    @Override
    public void init() {
        leftServo = hardwareMap.get(ServoImplEx.class, "leftOutake");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightOutake");




        if(rightDirection == 0) {
            rightServo.setDirection(Servo.Direction.FORWARD);
        } else if(rightDirection == 1) {
            rightServo.setDirection(Servo.Direction.REVERSE);
        }

        if(leftDirection == 0) {
            leftServo.setDirection(Servo.Direction.FORWARD);
        } else if(leftDirection == 1) {
            leftServo.setDirection(Servo.Direction.REVERSE);
        }

        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }


    @Override
    public void loop() {
        //Open left
        px = cx;
        cx = gamepad1.x;
        if (cx && !px) {
            leftServo.setPosition(0);
            rightServo.setPosition(0);
        }

        //Close left
        pa = ca;
        ca = gamepad1.a;
        if (ca && !pa) {
            leftServo.setPosition(leftServoAngle);
            rightServo.setPosition(rightServoAngle);
        }
    }
}