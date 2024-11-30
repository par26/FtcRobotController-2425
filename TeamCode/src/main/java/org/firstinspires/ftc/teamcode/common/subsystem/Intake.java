package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.action.Actions;
import org.firstinspires.ftc.teamcode.common.action.ParallelAction;
import org.firstinspires.ftc.teamcode.common.action.RunAction;
import org.firstinspires.ftc.teamcode.common.utils.RobotConstants;


public class Intake {
    public enum State{
        FORWARD, REVERSE, STOP
    }

    public CRServo spin;
    private State state;
    private Servo larmPivot;
    private Servo rarmPivot;

    public RunAction spinIntake, stopIntake, reverseIntake, lowerArm, retractArm, twistArm;
    private double spinPower, reversePower, stopPower;
    //adjust as needed, for the wheel

    private final double ARM_LOWER, ARM_RETRACT;
  //palceholder arm and wrist values
    public Intake(HardwareMap hardwareMap) {
        spin = hardwareMap.get(CRServo.class, "intakeSpin");
        larmPivot = hardwareMap.get(Servo.class, "leftArmPivot");
        rarmPivot = hardwareMap.get(Servo.class, "rightArmPivot");
        this.state = State.STOP;
        ARM_LOWER = RobotConstants.INTAKE_ARM_LOWER;
        ARM_RETRACT = RobotConstants.INTAKE_ARM_RETRACT;


        spinIntake = new RunAction(this::spinIntake);
        reverseIntake = new RunAction(this::reverseIntake);
        stopIntake = new RunAction(this::stopIntake);
        lowerArm = new RunAction(this::lowerArm);
        retractArm = new RunAction(this::retractArm);


    }
    //okok
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


    public void init() {
        Actions.runBlocking(new ParallelAction(retractArm, stopIntake));

    }
    public void start() {
        Actions.runBlocking(new ParallelAction(retractArm, stopIntake));
    }



}