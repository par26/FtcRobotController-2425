package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Extend;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestExtend extends OpMode {

    Extend extend;

    boolean ca, pa;
    int count = 0;

    @Override
    public void init() {
        extend = new Extend(hardwareMap);
    }

    @Override
    public void start() {
        extend.init();
        extend.start();
    }

    @Override
    public void loop() {

        pa = ca;
        ca = gamepad1.a;
        if (ca && !pa) {
            extend.switchExtendState();
            telemetry.addLine("Extend Switched" + extend.getState());
            count++;
            telemetry.update();
        }
    }
}
