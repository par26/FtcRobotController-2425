package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.action.Actions;
import org.firstinspires.ftc.teamcode.common.action.SequentialAction;
import org.firstinspires.ftc.teamcode.common.autonomous.Auton;
import org.firstinspires.ftc.teamcode.common.autonomous.FieldConstants;

@Autonomous
public class BlueBucket extends LinearOpMode {

    Auton auton;

    @Override
    public void runOpMode() {
        auton = new Auton(hardwareMap, FieldConstants.RobotStart.BLUE_BUCKET, new Follower(hardwareMap));

        Actions.runBlocking(
                new SequentialAction(
                        auton.depositPreload(),
                        auton.depositBucket(),
                        auton.park()
                )
        );

    }
}
