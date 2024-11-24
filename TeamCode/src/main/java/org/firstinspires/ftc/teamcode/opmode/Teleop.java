package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.common.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.common.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.common.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.common.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.pedroPathing.follower.Follower;

import java.util.Arrays;
import java.util.List;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class Teleop extends OpMode  {


    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

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


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    public static double rightOpen = 0.5;

    public static double leftClose = .2;

    public static double rightClose = .2;

    @Override
    public void init() {


        //follower.initialize();

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //leftRear.setDirection(DcMotorEx.Direction.REVERSE);


        //rightElbowServo = hardwareMap.get(ServoImplEx.class, "rightElbowServo");
        //leftElbowServo = hardwareMap.get(ServoImplEx.class, "leftElbowServo");

        //rightElbowServo.scaleRange(0, MAX_RETRACT_ANGLE);
        //leftElbowServo.scaleRange(0, MAX_RETRACT_ANGLE);

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        leftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);



        leftSlide.setDirection(DcMotorEx.Direction.REVERSE);

        /*eftClaw = hardwareMap.get(Servo.class, "leftClaw");

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        leftClaw.setDirection(Servo.Direction.REVERSE);

        leftClaw.setPosition(1);
        rightClaw.setPosition(1);

        clawClosed = false;

        leftServo = hardwareMap.get(ServoImplEx.class, "leftServo");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightServo");




        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));


        //+rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.REVERSE);

        leftServo.setPosition(0.006);
        rightServo.setPosition(0.006); */

        claw = hardwareMap.get(ServoImplEx.class, "claw");
        claw.setDirection(Servo.Direction.REVERSE);

        leftServo = hardwareMap.get(ServoImplEx.class, "leftServo");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightServo");


        wrist = hardwareMap.get(ServoImplEx.class, "wrist");



        leftServo.setDirection(Servo.Direction.REVERSE);


        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));


        //leftServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setPosition(0);
        rightServo.setPosition(0);

        wrist.setPosition(0);



    }

    @Override
    public void loop() {


        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // this is strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.85);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);




        if(Math.abs(claw.getPosition() - 0.5) < .01) {
            clawClosed = true;
        } else {
            clawClosed = false;
        }

        if(Math.abs(wrist.getPosition()- 0.666) < .02) {
            wristTwisted = true;
        } else {
            wristTwisted = false;
        }


        if (currentGamepad2.a && !previousGamepad1.a) {
            if(clawClosed) {
                claw.setPosition(0);
            } else {
                claw.setPosition(0.5);
                //clawClosed = true;
            }
        }


        //Open left
        if (currentGamepad1.x && !previousGamepad1.x) {
            if(wristTwisted) {
                wrist.setPosition(0.0);
            } else {
                wrist.setPosition(.666666);
                //clawClosed = true;
            }
        }



        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            claw.setPosition(0.5);

        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            claw.setPosition(0);

        }



        if (currentGamepad1.b && !previousGamepad1.b) {
            leftServo.setPosition(1.0);
            rightServo.setPosition(1.0);
        }


        if (currentGamepad1.y && !previousGamepad1.y) {
            leftServo.setPosition(0.006);
            rightServo.setPosition(0.006);
        }



        double rightSlidePower = Range.clip(gamepad1.right_trigger, 0, 0.85);
        rightSlide.setPower(rightSlidePower);

        double leftSlidePower = Range.clip(gamepad1.left_trigger, 0, 0.85);
        leftSlide.setPower(-leftSlidePower);

        //powerSlides(slidePower);


       // telemetry.addData("Claw close value", clawClosed);
        telemetry.update();

    }


    @Override
    public void stop() {
        

    }





    public void moveSlidesToPostion(int targetPosition) {
        while(leftSlide.getCurrentPosition() != targetPosition) {
            leftSlide.setTargetPosition(targetPosition);
        }
    }

}
