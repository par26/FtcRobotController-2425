package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.common.action.RunAction;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;

public class Lift {

    private Telemetry telemetry;

    public DcMotorEx liftMotor;
    public DcMotorEx liftMotor2;

    private int pos, initalPos;
    public RunAction setPositionLow; //note that you can make more runactions, very easy
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

    public Lift (HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift1");
        liftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        liftMotor2 = hardwareMap.get(DcMotorEx.class, "lift2");
        liftMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        liftPID = new PIDController(p, i, d);

        setPositionLow = new RunAction(this::setPosition);
    }



    public void updatePID() {
        setPower(getLiftPID(getCurrentPos(), targetPos));
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
        target = b;
    }

    public void setPosition() {
        setTarget(0); //this will be used as template to move Lif tto where we want the lift to move
    }


    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        // cache motor powers to prevent unnecessary writes
        if(Math.abs(power - lastPower) > 0.02) {
            if(power > 0) {
                liftMotor.setPower(power);
            } else {
                liftMotor2.setPower(Math.abs(power));
            }
            lastPower = power;
        }
    }


    public void setTargetHeight(double inches) {
        setTarget(toTicks(inches));
    }

    //util kinda

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
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init() {
        resetEncoder();
        initalPos = currentPos;
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        liftMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void start() {
        initalPos = currentPos;
        setTarget(10);
    }

}