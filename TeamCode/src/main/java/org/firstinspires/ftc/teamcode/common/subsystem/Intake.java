package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.action.RunAction;

public class Intake {
    public enum State{
        FORWARD, REVERSE, STOP
    }

    public CRServo spin;
    private State state;
    private Servo larmPivot;
    private Servo rarmPivot;
    private Servo wrist;
    public RunAction spinIntake, stopIntake, reverseIntake, lowerArm, retractArm, twistArm;
    private double spinPower, reversePower, stopPower;
    //adjust as needed, for the wheel

    private final double ARM_LOWER, ARM_RETRACT, wristServoPosition;
  //palceholder arm and wrist values
    public Intake(HardwareMap hardwareMap, State state, double armLower, double armRetract, double wristServoPosition) {
        spin = hardwareMap.get(CRServo.class, "intakeSpin");
        larmPivot = hardwareMap.get(Servo.class, "leftArmPivot");
        rarmPivot = hardwareMap.get(Servo.class, "rightArmPivot");
        wrist = hardwareMap.get(Servo.class, "intakeWrist");
        this.state = state;
        ARM_LOWER = armLower;
        ARM_RETRACT = armRetract;
        this.wristServoPosition = wristServoPosition;

        spinIntake = new RunAction(this::spinIntake);
        reverseIntake = new RunAction(this::reverseIntake);
        stopIntake = new RunAction(this::stopIntake);
        lowerArm = new RunAction(this::lowerArm);
        retractArm = new RunAction(this::retractArm);
        twistArm = new RunAction(this::twistArm);

    }
    //o
    //spin to win part
    public void setSpin(State state, boolean onlyChangeState) {
        if (onlyChangeState) {
            this.state = state;
            //we are only changing the state, not actually changing the power
        } else {
            if (state == state.FORWARD) {
                spinIntake();
            } else if (state == state.REVERSE) {
                reverseIntake();
            } else if (state == state.STOP) {
                stopIntake();
            }
        }
    }

    public void spinIntake() {
        spin.setPower(spinPower);
        this.state = state.FORWARD;
    }

    public void reverseIntake() {
        spin.setPower(reversePower);
        this.state = state.REVERSE;
    }

    public void stopIntake() {
        spin.setPower(stopPower);
        this.state = state.STOP;
    }

    // the arm itself

    public void lowerArm() {


        larmPivot.setPosition(ARM_LOWER);
       rarmPivot.setPosition(ARM_LOWER);
    }
    public void retractArm() {
        //bw
       larmPivot.setPosition(ARM_RETRACT);
       rarmPivot.setPosition(ARM_RETRACT);
    }
    public void twistArm() {
      wrist.setPosition(wristServoPosition);
    }
/*
    public void init() {
        Actions.runBlocking(new ParallelAction(pivotTransfer, spinStop));

    }
    public void start() {
        Actions.runBlocking(new ParallelAction(pivotTransfer, spinStop));
    }
*/
}