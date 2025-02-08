package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Extend;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestExtend extends OpMode {

    Extend extend;
    Intake intake;

    boolean ca, pa, px, cx, cy, py;
    int count = 0;

    @Override
    public void init() {
        extend = new Extend(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        extend.start();
        intake.start();
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

        px = cx;
        cx = gamepad1.x;
        if (cx && !px) {
            intake.lowerArm();
        }

        py = cy;
        cy = gamepad1.y;
        if (cy && !py) {
            intake.retractArm();
        }
    }
}
