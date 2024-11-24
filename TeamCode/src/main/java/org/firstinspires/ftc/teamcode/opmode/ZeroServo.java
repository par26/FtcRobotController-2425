package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ZeroServo extends OpMode {


    ServoImplEx servo;

    //Telemetry telemetry;
    @Override
    public void init() {
        servo = hardwareMap.get(ServoImplEx.class, "servo");

        telemetry.addData("Servo Current Position: ", servo.getPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        servo.setPosition(0);
        telemetry.update();
    }



}