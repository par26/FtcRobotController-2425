package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.opmode.Teleop;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestIntake extends OpMode {

    public static double leftServoPower = 1;
    public static double rightServoPower = 1;

    CRServo leftServo, rightServo;

    @Override
    public void init() {
        leftServo = hardwareMap.get(CRServo.class, "leftSpin");
        rightServo = hardwareMap.get(CRServo.class, "rightSpin");

        rightServo.setDirection(CRServo.Direction.REVERSE);
    }


    @Override
    public void loop() {
        leftServo.setPower(leftServoPower);
        rightServo.setPower(rightServoPower);
    }

}