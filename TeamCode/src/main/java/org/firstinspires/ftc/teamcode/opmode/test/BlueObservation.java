package org.firstinspires.ftc.teamcode.opmode.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.action.Actions;
import org.firstinspires.ftc.teamcode.common.action.ParallelAction;
import org.firstinspires.ftc.teamcode.common.action.SequentialAction;
import org.firstinspires.ftc.teamcode.common.action.SleepAction;
import org.firstinspires.ftc.teamcode.common.autonomous.Auton;
import org.firstinspires.ftc.teamcode.common.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.common.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.common.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Lift;
import org.firstinspires.ftc.teamcode.common.subsystem.Outake;

@Autonomous
public class BlueObservation extends LinearOpMode {

    Lift lift;
    Outake outake;

    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);
        lift = new Lift(hardwareMap);
        outake = new Outake(hardwareMap);
        lift.start();
        //outake.start();


        waitForStart();


        Actions.runBlocking(new SequentialAction(lift.topBucket, lift.waitSlide(), new SleepAction(3000), outake.openClaw, lift.lowered, lift.waitSlide()));

    }
}
