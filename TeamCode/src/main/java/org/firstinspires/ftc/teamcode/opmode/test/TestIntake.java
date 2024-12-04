package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.Teleop;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestIntake extends Teleop {

    static double leftServoPower = -1;
    static double rightServoPower = 1;

    CRServo leftServo, rightServo;

    @Override
    public void init() {
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
    }


    @Override
    public void loop() {
        leftServo.setPower(leftServoPower);
        rightServo.setPower(rightServoPower);
    }

}