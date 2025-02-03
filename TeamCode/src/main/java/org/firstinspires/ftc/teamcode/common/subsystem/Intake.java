package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.action.Actions;
import org.firstinspires.ftc.teamcode.common.action.ParallelAction;
import org.firstinspires.ftc.teamcode.common.action.RunAction;
import org.firstinspires.ftc.teamcode.common.utils.RobotConstants;


public class Intake {
    public enum IntakeState {
        FORWARD, REVERSE, STOP
    }
    public enum PivotState {
        LOWER, RETRACT
    }


    public CRServo Lspin, Rspin;
    public IntakeState intakeState;
    public static PivotState pivotState;
    private ServoImplEx larmPivot;
    private ServoImplEx rarmPivot;

    public RunAction intakeSpin, intakeStop, intakeReverse, armLower, armToTransfer;
    private double spinPower, reversePower, stopPower;
    //adjust as needed, for the wheel

    private final double ARM_LOWER, ARM_RETRACT;
    //palceholder arm and wrist values
    public Intake(HardwareMap hardwareMap) {
        Lspin = hardwareMap.get(CRServo.class, "leftSpin");
        Rspin = hardwareMap.get(CRServo.class, "rightSpin");
        Rspin.setDirection(CRServo.Direction.REVERSE);
        larmPivot = hardwareMap.get(ServoImplEx.class, "leftArmPivot");
        rarmPivot = hardwareMap.get(ServoImplEx.class, "rightArmPivot");
        this.intakeState = IntakeState.STOP;
        pivotState = PivotState.RETRACT;
        ARM_LOWER = RobotConstants.INTAKE_ARM_LOWER;
        ARM_RETRACT = RobotConstants.INTAKE_ARM_RETRACT;

        spinPower = RobotConstants.intakeSpinInPwr;
        reversePower = RobotConstants.intakeSpinOutPwr;
        stopPower = 0;

        larmPivot.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rarmPivot.setPwmRange(new PwmControl.PwmRange(500, 2500));

        rarmPivot.setDirection(ServoImplEx.Direction.REVERSE);

        intakeSpin = new RunAction(this::spinIntake);
        intakeReverse = new RunAction(this::reverseIntake);
        intakeStop = new RunAction(this::stopIntake);
        armLower = new RunAction(this::lowerArm);
        armToTransfer = new RunAction(this::retractArm);


    }

    public void start() {
        Actions.runBlocking(new ParallelAction(armToTransfer, intakeStop));
    }


    //okok
    //spin to win part
    public void setSpin(IntakeState state, boolean onlyChangeState) {
        if (onlyChangeState) {
            this.intakeState = state;
            //we are only changing the state, not actually changing the power
        } else {
            if (state == IntakeState.FORWARD) {
                spinIntake();
            } else if (state == IntakeState.REVERSE) {
                reverseIntake();
            } else if (state == IntakeState.STOP) {
                stopIntake();
            }
        }
    }



    public void setPower(double power) {
        Lspin.setPower(power);
        Rspin.setPower(power);
    }

    public void spinIntake() {
        Lspin.setPower(spinPower);
        Rspin.setPower(spinPower);
        this.intakeState = IntakeState.FORWARD;
    }

    public void reverseIntake() {
        Lspin.setPower(reversePower);
        Rspin.setPower(reversePower);
        intakeState = IntakeState.REVERSE;
    }

    public void stopIntake() {
        Lspin.setPower(stopPower);
        Rspin.setPower(stopPower);
        intakeState = IntakeState.STOP;
    }

    // the arm itself
    public void lowerArm() {
        larmPivot.setPosition(ARM_LOWER);
       rarmPivot.setPosition(ARM_LOWER);
       pivotState = PivotState.LOWER;
    }
    public void retractArm() {
        //bw
       larmPivot.setPosition(ARM_RETRACT);
       rarmPivot.setPosition(ARM_RETRACT);
       pivotState = PivotState.RETRACT;
    }

    public void zeroArm() {
        larmPivot.setPosition(0);
        rarmPivot.setPosition(0);
    }


}