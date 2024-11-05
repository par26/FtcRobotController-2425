package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.common.RobotHardware;


public class Lift implements Subsystem {

    //private final RobotHardware robot;

    public static double
            kP = 0.0031, kI = 0, kD = 0.0001, kF = 0.00, // PID values


    TICKS_PER_REV = 384.5, // ticks
            PULLEY_CIRCUMFERENCE = 4.40945; // inches

    public static int
            MIN_POS = 0, MAX_POS = 2500,
            PID_TOLERANCE = 2;// ticks

    enum State {
        PID, MANUAL
    }

    State state;
    PDController liftPID;

    double PID;

    double power;
    double lastPower;
    int targetPos;
    int currentPos;

    double targetVelocity;
    double currentVelocity;

    public Lift(HardwareMap hardwareMap) {

        //.robot = RobotHardware.getInstance();

        state = State.PID;

        targetPos = 0;
        targetVelocity = 0;
        currentVelocity = 0;

        liftPID = new PDController(kP, kD);
    }

    public void read() {
        currentPos =  leftMotor.getCurrentPosition();
        currentVelocity = leftMotor.getVelocity();
    }

    public void write() {
        PID = getLiftPID(currentPos, targetPos);

        switch(state) {
            case PID:
                if (getAbsPosError() < PID_TOLERANCE) {
                    power = 0;
                } else {
                    power = PID;
                }
                break;
            case MANUAL:
                // set manual power elsewhere
                targetPos = currentPos;
//                setTargetPos(currentPos + getDecelDelta()); // update target position
                break;
        }

        setPower(power);
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


    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        // cache motor powers to prevent unnecessary writes
        if(Math.abs(power - lastPower) > 0.02) {
            liftMotor.setPower(power);
            liftMotor2.setPower(power);
            lastPower = power;
        }
    }

    public void setTargetPos(int pos) {
        state = State.PID;
        int newTargetPos = Range.clip(pos, MIN_POS, MAX_POS);
//        if(newTargetPos != targetPos) liftPID.reset();
        targetPos = newTargetPos;
    }

    public void setTargetHeight(double inches) {
        setTargetPos(toTicks(inches));
    }



    public void resetEncoder() {
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updatePID() {
        liftPID.setP(kP);
        liftPID.setI(kI);
        liftPID.setD(kD);
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

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("lift state", state);
        telemetry.addData("targetPos", targetPos);
        telemetry.addData("currentPos", currentPos);
        telemetry.addData("power", power);
    }

}