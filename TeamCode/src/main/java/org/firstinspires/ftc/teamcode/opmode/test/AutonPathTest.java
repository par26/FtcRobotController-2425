package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.action.Actions;
import org.firstinspires.ftc.teamcode.common.action.SequentialAction;
import org.firstinspires.ftc.teamcode.common.action.SleepAction;
import org.firstinspires.ftc.teamcode.common.autonomous.Auton;
import org.firstinspires.ftc.teamcode.common.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.common.pedroPathing.FollowPathAction;
import org.firstinspires.ftc.teamcode.common.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.common.pedroPathing.constants.LConstants;

@Autonomous
public class AutonPathTest extends LinearOpMode {

    Auton auton;

    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);

        waitForStart();

        auton = new Auton(hardwareMap, FieldConstants.RobotStart.TEST, new Follower(hardwareMap));

        Actions.runBlocking(
                new SequentialAction(
                        auton.testPath()
                )
        );

    }
}
