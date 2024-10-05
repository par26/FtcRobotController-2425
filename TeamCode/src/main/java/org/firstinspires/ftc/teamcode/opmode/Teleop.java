package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class Teleop extends CommandOpMode {

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();


    }

    @Override
    public void run() {

    }
}
