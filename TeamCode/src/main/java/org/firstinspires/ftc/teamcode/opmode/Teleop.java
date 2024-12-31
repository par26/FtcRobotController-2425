package org.firstinspires.ftc.teamcode.opmode;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.action.Actions;

import org.firstinspires.ftc.teamcode.common.subsystem.Extend;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.subsystem.Lift;
import org.firstinspires.ftc.teamcode.common.subsystem.Outake;

import java.util.Arrays;
import java.util.List;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class Teleop extends OpMode  {


    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;


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
    //Extend extend;
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


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    Intake.State teleopState;

    double intakePower = 0;

    public static double rightOpen = 0.5;

    public static double leftClose = .2;

    public static double rightClose = .2;

    @Override
    public void init() {


        //follower.initialize();



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

        teleopState = Intake.State.STOP;

        lift = new Lift(hardwareMap);
        //extend = new Extend(hardwareMap);
        outake = new Outake(hardwareMap);
        intake = new Intake(hardwareMap);


        //leftRear.setDirection(DcMotorEx.Direction.REVERSE);


        //rightElbowServo = hardwareMap.get(ServoImplEx.class, "rightElbowServo");
        //leftElbowServo = hardwareMap.get(ServoImplEx.class, "leftElbowServo");

        //rightElbowServo.scaleRange(0, MAX_RETRACT_ANGLE);
        //leftElbowServo.scaleRange(0, MAX_RETRACT_ANGLE);




        transferState = 1;

        initSubsystems();
        startSubsystems();


        intake.zeroArm();

    }


    private void initSubsystems() {
        lift.init();
        //extend.init();
        outake.init();
        intake.init();
    }

    private void startSubsystems() {
        lift.start();
        //extend.start();
        outake.start();
        intake.start();
    }

    @Override
    public void loop() {


        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        double y = -gamepad2.left_stick_y; // Remember, this is reversed!
        double x = gamepad2.left_stick_x; // this is strafing
        double rx = gamepad2.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.85);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;



        if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up)  {
            intake.setPower(1);
            intake.setSpin(Intake.State.FORWARD, true);
        }

        if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down)  {
            intake.setPower(-1);
            intake.setSpin(Intake.State.REVERSE, true);
        }

        if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            intake.setPower(0);
            intake.setSpin(Intake.State.FORWARD, true);
        }




        switch(transferState) {
            case 0:
                intake.lowerArm();
                outake.toTransfer();
                transferState = 1;
                break;
            case 1:
                intake.retractArm();
                transferState = 2;
                break;
            case 2:
                if(currentGamepad1.x && !previousGamepad1.x) {
                    outake.closeClaw();
                }
                transferState = 3;
                break;
            case 3:
                outake.toBucket();
                break;
            default:
                transferState = 0;

        }


        if(currentGamepad1.y && !previousGamepad1.y) {
            transferState = 0;
        }





        if(currentGamepad1.x && previousGamepad1.x) {
            outake.toTransfer();
        }


        if(currentGamepad1.y && previousGamepad1.y) {
            outake.toBucket();
        }



        if (currentGamepad2.a && !previousGamepad1.a) {
           outake.switchClawState();
        }


//        //Open left
//        if (currentGamepad1.x && !previousGamepad1.x) {
//            outake.switchWristState();
//        }



        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            //extend.setManualPower(0.7);
            intake.retractArm();
        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            //extend.setManualPower(-0.7);
            intake.lowerArm();
        }







        double slidePower = gamepad1.right_trigger - gamepad1.left_trigger;

        if(Math.abs(slidePower) > arm_deadband) {
            lift.setPower(slidePower);
        } else {
            lift.setPower(.1);
        }



        //powerSlides(slidePower);
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

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
