package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.action.Actions;
import org.firstinspires.ftc.teamcode.common.action.ParallelAction;
import org.firstinspires.ftc.teamcode.common.action.RunAction;
import org.firstinspires.ftc.teamcode.common.utils.RobotConstants;


public class Intake {
    public enum State{
        FORWARD, REVERSE, STOP
    }

    public CRServo Lspin, Rspin;
    private State state;
    private ServoImplEx larmPivot;
    private ServoImplEx rarmPivot;

    public RunAction spinIntake, stopIntake, reverseIntake, lowerArm, retractArm, twistArm;
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
        this.state = State.STOP;
        ARM_LOWER = RobotConstants.INTAKE_ARM_LOWER;
        ARM_RETRACT = RobotConstants.INTAKE_ARM_RETRACT;


        larmPivot.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rarmPivot.setPwmRange(new PwmControl.PwmRange(500, 2500));

        rarmPivot.setDirection(ServoImplEx.Direction.REVERSE);

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





    public void setPower(double power) {
        Lspin.setPower(power);
        Rspin.setPower(power);
    }

    public void spinIntake() {
        Lspin.setPower(spinPower);
        Rspin.setPower(spinPower);
        this.state = state.FORWARD;
    }

    public void reverseIntake() {
       Lspin.setPower(reversePower);
        Rspin.setPower(reversePower);
        this.state = state.REVERSE;
    }

    public void stopIntake() {
        Lspin.setPower(stopPower);
        Rspin.setPower(stopPower);
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

    public void zeroArm() {
        larmPivot.setPosition(0);
        rarmPivot.setPosition(0);
    }


    public void init() {
        Actions.runBlocking(new ParallelAction(retractArm, stopIntake));

    }
    public void start() {
        Actions.runBlocking(new ParallelAction(retractArm, stopIntake));
    }



}