package org.firstinspires.ftc.teamcode.opmode;



import static org.firstinspires.ftc.teamcode.common.utils.RobotConstants.intakeSpinInPwr;
import static org.firstinspires.ftc.teamcode.common.utils.RobotConstants.intakeSpinOutPwr;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.action.Actions;

import org.firstinspires.ftc.teamcode.common.action.ParallelAction;
import org.firstinspires.ftc.teamcode.common.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.common.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Extend;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.subsystem.Lift;
import org.firstinspires.ftc.teamcode.common.subsystem.Outake;

import java.util.List;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class Teleop extends OpMode  {


    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;



    private Follower follower;

    private int transferState;



    private final double MAX_RETRACT_ANGLE = 200.0/360.0;

    boolean clawClosed;
    boolean elbowExtended;

    boolean wristTwisted;

    public static double idle_power = 0.0;

    ServoImplEx leftServo;
    ServoImplEx rightServo;
    DcMotorEx leftSlideMotor;

    double clawOpen = 0;
    double clawClose = 0.4;

    Servo wrist;

    DcMotorEx rightSlideMotor;



    Lift lift;
    Extend extend;
    Outake outake;
    Intake intake;

    //ServoImplEx leftElbowServo;
    //ServoImplEx rightElbowServo;

    //DcMotorEx leftSlide;
    final double arm_deadband = 0.05;

    boolean pb, cb = false;

    //rising edge for the a button
    boolean pa, ca = false;

    //rising edge for the x button
    boolean px, cx = false;

    //rising edge for the y button
    boolean py, cy = false;


    //rising edge for left trigger
    boolean plt, clt = false;

    //rising edge for right trigger
    boolean prt, crt = false;

    //rising edge for the left bumper
    boolean plb, clb = false;

    //rising edge for right bumper
    boolean prb, crb = false;

    //rising edge for back button
    boolean pbb, cbb = false;

    //rising edge for start button
    boolean psb, csb = false;

    private List<DcMotorEx> motors;

    Servo claw;
    Servo rightClaw;

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    private final Pose startPose = new Pose(0,0,0);

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    Intake.IntakeState teleopState;

    double intakePower = 0;

    public static double rightOpen = 0.5;

    public static double leftClose = .2;

    public static double rightClose = .2;

    @Override
    public void init() {


        //follower.initialize();


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);



        teleopState = Intake.IntakeState.STOP;

        lift = new Lift(hardwareMap);
        extend = new Extend(hardwareMap);
        outake = new Outake(hardwareMap);
        intake = new Intake(hardwareMap);


    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        startSubsystems();
    }

    private void startSubsystems() {
        lift.start();
        extend.start();
        outake.start();
        intake.start();

       outake.toTransfer();
    }

    @Override
    public void loop() {


        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);



        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]


        if (gamepad1.b) {
            intake.setPower(0);
            intake.setSpin(Intake.IntakeState.STOP, true);
        } else if (gamepad1.dpad_down) {
            intake.setPower(intakeSpinOutPwr);
            intake.setSpin(Intake.IntakeState.FORWARD, true);
        } else {
            intake.setPower(intakeSpinInPwr);
            intake.setSpin(Intake.IntakeState.REVERSE, true);

        }


//
//        switch(transferState) {
//            case 0:
//
//                intake.lowerArm();
//                //outake.toTransfer();
//                transferState = 1;
//                break;
//            case 1:
//                intake.retractArm();
//                transferState = 2;
//                break;
//            case 2:
//                if(currentGamepad1.x && !previousGamepad1.x) {
//                    //outake.closeClaw();
//                }
//                transferState = 3;
//                break;
//            case 3:
//                //outake.toBucket();
//                break;
//            default:
//                transferState = 0;
//
//        }
//
//
//        if(currentGamepad1.y && !previousGamepad1.y) {
//            transferState = 0;
//        }
//
//
//
//
//
        if(currentGamepad2.x && previousGamepad2.x) {
            outake.toTransfer();
        }


        if(currentGamepad2.y && previousGamepad2.y) {
            Actions.runBlocking(new ParallelAction(outake.toBucket, intake.armLower));
        }
//
//
//
        if (currentGamepad2.a && !previousGamepad2.a) {
            telemetry.addLine("Claw button pressed");
           outake.switchClawState();
        }







        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            //extend.setManualPower(0.7);
            intake.retractArm();
        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            //extend.setManualPower(-0.7);
            intake.lowerArm();
        }









        double liftPower = gamepad2.right_trigger - gamepad2.left_trigger;

        lift.setPower(-liftPower);


        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();



        // telemetry.addData("Claw close value", clawClosed);
        telemetry.update();

    }


    @Override
    public void stop() {
        intake.lowerArm();
    }





    public void moveSlidesToPostion(int targetPosition) {
        while(leftSlide.getCurrentPosition() != targetPosition) {
            leftSlide.setTargetPosition(targetPosition);
        }
    }

}
