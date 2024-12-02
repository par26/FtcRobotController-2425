package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.action.Action;
import org.firstinspires.ftc.teamcode.common.action.RunAction;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;

public class Lift {

    private Telemetry telemetry;

    public DcMotorEx rightLift;
    public DcMotorEx leftLift;

    private int pos, initalPos;
    public RunAction setPosition; //note that you can make more runactions, very easy
    public PIDController liftPID;
    public static int target;

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

    public Lift (HardwareMap hardwareMap) {
        rightLift = hardwareMap.get(DcMotorEx.class, "lift1");
        rightLift.setDirection(DcMotorEx.Direction.REVERSE);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftLift = hardwareMap.get(DcMotorEx.class, "lift2");
        leftLift.setDirection(DcMotorEx.Direction.REVERSE);
        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        liftPID = new PIDController(p, i, d);

        setPosition = new RunAction(this::setPosition);
    }


    public void update() {
        if (state == State.PID) {
            liftPID.setPID(p, i, d);

            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double pid = liftPID.calculate(rightLift.getCurrentPosition(), target);
            double ticks_in_degrees = 537.7 / 360.0;
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double ffpower = pid + ff;

            setPower(ffpower);

            telemetry.addData("lift pos", rightLift.getCurrentPosition());
            telemetry.addData("lift target", target);
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
        state = State.MANUAL;
        target = b;
    }

    public void setPosition() {
        setTarget(0); //this will be used as template to move Lif tto where we want the lift to move
    }


    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        // cache motor powers to prevent unnecessary writes
        if(Math.abs(power - lastPower) > 0.02) {
            rightLift.setPower(power);
            leftLift.setPower(power);
            lastPower = power;
        }
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
        return Math.abs(target - leftLift.getCurrentPosition()) < 10;
    }


    public void init() {
        resetEncoder();
        initalPos = currentPos;
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
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