package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.action.Action;
import org.firstinspires.ftc.teamcode.common.action.RunAction;

public class Extend {

    private Telemetry telemetry;

    public DcMotorEx rightMotor;
    public DcMotorEx leftMotor;

    private int pos, initalPos;
    public RunAction setPosition; //note that you can make more runactions, very easy
    public PIDController ExtendPID;
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

    public static double p = 0.04, i = 0, d = 0.000001, f = 0.01;
    private final double TICKS_PER_REV = 384.5; //ticks

    private final double PULLEY_CIRCUMFERENCE = 4.40945; //inches

    public Extend(HardwareMap hardwareMap) {

        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        rightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ExtendPID = new PIDController(p, i, d);

        setPosition = new RunAction(this::setPosition);
    }
    public void update() {
        if (state == Extend.State.PID) {

            ExtendPID.setPIDF(p, i, d, f);

            rightMotor
.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


            slidesReached = ExtendPID.atSetPoint();
            slidesRetracted = (target <= 0) && slidesReached;


            double pid = ExtendPID.calculate(rightMotor.getCurrentPosition(), target);

            // Just make sure it gets to fully retracted if target is 0
            if (target == 0 && !slidesReached) {
                power -= 0.1;
            } /*else if (target >= MAX_SLIDES_EXTENSION && !slidesReached) {
                power += 0.1;
            } */

            if (slidesRetracted) {
                resetEncoder();
                setPower(0);
            } else {
                setPower(pid);
            }


            telemetry.addData("Extend pos", rightMotor.getCurrentPosition());
            telemetry.addData("Extend target", target);
        } else {
            setPower(this.power);
        }
    }

    public double getExtendPID(double currentPos, double targetPos) {
        return Range.clip(ExtendPID.calculate(currentPos, targetPos), MAX_DOWN_POWER, MAX_UP_POWER);
    }



    public void setManualPower(double power) {
        state = Extend.State.MANUAL;
        this.power = power;
    }

    public void stopManual() {
        state = Extend.State.PID;
        this.power = 0;
    }

    public void setTarget(int b) {
        state = Extend.State.PID;
        target = b;
    }

    public void setPosition() {
        setTarget(0); //this will be used as template to move Lif tto where we want the Extend to move
    }


    public void setPower(double power) {

        power = Range.clip(power, -1, 1);

        if(Math.abs(lastPower- power) > 0.01) {
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }


        lastPower = power;

    }


    public void setTargetHeight(double inches) {
        setTarget(toTicks(inches));
    }

    //util kinda
    public void updatePIDConstants(double p, double d, double i, double f) {
        ExtendPID.setPIDF(p, i, d, f);
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
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean atTarget() {
        return Math.abs(target - leftMotor.getCurrentPosition()) < 10;
    }


    public void init() {
        resetEncoder();
        initalPos = currentPos;
        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        resetEncoder();

    }

    public void start() {
        initalPos = currentPos;
        setTarget(0);
    }


    public Action waitSlide() {
        return new Action() {
            private boolean set = false;
            @Override
            public boolean run(TelemetryPacket telemetryPacket) {
                return atTarget();
            }
        };
    }


}