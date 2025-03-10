package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.action.Action;
import org.firstinspires.ftc.teamcode.common.action.RunAction;

import org.firstinspires.ftc.teamcode.common.utils.RobotConstants;

public class Lift {

    private Telemetry telemetry;

    public DcMotorEx rightLift;
    public DcMotorEx leftLift;

    private int pos, initalPos;
    public RunAction update, topBucket, lowBucket, l2Touch, l1Touch, lowered; //note that you can make more runactions, very easy
    public PIDFController liftPID;
    public static int target;


    private boolean slidesReached;

    private boolean slidesRetracted;

    private double power;
    private double lastPower;
    private int targetPos;
    private int currentPos;



    private final double MAX_DOWN_POWER = 1, MAX_UP_POWER = 0, MIN_POS = 0 , MAX_POS = 1; //hello import these into robot constants pls :)

    public enum State{
        PID, MANUAL
    }

    State state;

    public static double p = 1, i = 0, d = 0, f = 0.01;
    private final double TICKS_PER_REV = 384.5; //ticks
    private final double PULLEY_CIRCUMFERENCE = 4.40945; //inches

    public Lift (HardwareMap hardwareMap) {
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        //rightLift.setDirection(DcMotorEx.Direction.REVERSE);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");

        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftLift.setDirection(DcMotorEx.Direction.REVERSE);

        liftPID = new PIDFController(p, i, d, f);

        topBucket = new RunAction(this::setTopBucket);
        lowBucket = new RunAction(this::setLowBucket);
        l2Touch = new RunAction(this::setL2Touch);
        l1Touch = new RunAction(this::setL1Touch);
        lowered = new RunAction(this::setLowered);
        update = new RunAction(this::update);
    }

    public void start() {
        resetEncoder();
        initalPos = currentPos;
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        resetEncoder();

        setTarget(0);
    }

    public void update() {
        if (state == State.PID) {

            liftPID.setPIDF(p, i, d, f);

            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


            double pid = liftPID.calculate(rightLift.getCurrentPosition(), target);

//
//            // Just make sure it gets to fully retracted if target is 0
//            if (slidesRetracted&& !slidesReached) {
//                power -= .05;
//            } /*else if (target >= MAX_SLIDES_EXTENSION && !slidesReached) {
//                power += 0.1;
//            } */
//
//            if (slidesRetracted) {
//                resetEncoder();
//                setPower(0);
//            } else {
                setPower(pid);
            //}

        } else {
            setPower(this.power);
        }
    }

    public double getLiftPID(double currentPos, double targetPos) {
        return Range.clip(liftPID.calculate(currentPos, targetPos), MAX_DOWN_POWER, MAX_UP_POWER);
    }

    public void setManualPower(double power) {
        state = State.MANUAL;
        this.power = power;
    }

    public void stopManual() {
        state = State.PID;
        this.power = 0;
    }

    public void setTarget(int b) {
        state = State.PID;
        target = b;
    }

    public void setPosition() {
        setTarget(0); //this will be used as template to move Lif tto where we want the lift to move
    }


    public void setPower(double power) {

        power = Range.clip(power, -.8, .8);

        if(Math.abs(lastPower- power) > 0.01) {
            leftLift.setPower(power);
            rightLift.setPower(power);
        }


        lastPower = power;

    }


    public void setTargetHeight(double inches) {
        setTarget(toTicks(inches));
    }

    //util kinda
    public void updatePIDConstants(double p, double d, double i, double f) {
        liftPID.setPIDF(p, i, d, f);
    }



    public int getTargetPos() {
        return targetPos;
    }

    public int getCurrentPos() {
        return currentPos;
    }

    public int getAbsPosError() {
        return Math.abs(targetPos - currentPos);
    }

    public int toTicks(double inches) { // convert inches to motor ticks
        return (int) (inches / PULLEY_CIRCUMFERENCE * TICKS_PER_REV);
    }

    public double toInches(double ticks) { // convert motor ticks to inches
        return ticks * PULLEY_CIRCUMFERENCE / TICKS_PER_REV;
    }

    public void resetEncoder() {
        rightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean atTarget() {
        return Math.abs(target - rightLift.getCurrentPosition()) < 10;
    }


    public Action waitSlide() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(TelemetryPacket telemetryPacket) {

                if(!set) {
                    telemetryPacket.addLine("code staarted");
                    setTarget(target);
                    set = true;
                }


                if (atTarget())
                {return false;}

                update();

                telemetryPacket.addLine("running wait action");
                telemetryPacket.addTimestamp();

                return true;
            }
        };
    }

    public void setTopBucket() {
        setTarget(RobotConstants.liftToHighBucket);
    }

    public void setLowBucket() {
        setTarget(RobotConstants.liftToLowBucket);
    }

    public void setLowered() {
        setTarget(RobotConstants.liftToTransfer);
    }

    public void setL1Touch() {
        setTarget(RobotConstants.liftToLowChamber);
    }

    public void setL2Touch() {
        setTarget(RobotConstants.liftToMidChamber);
    }


}