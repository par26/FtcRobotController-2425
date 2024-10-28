package org.firstinspires.ftc.teamcode.common.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.action.RunAction;

public class Outake {

    public enum clawState {
        CLOSED, OPEN
    }

    private clawState state;
    private ServoImplEx oClaw;
    private ServoImplEx lOutake;
    private ServoImplEx rOutake;
    private ServoImplEx outakeWrist;

    private double clawOpen, clawClosed;

    double wristPos, clawPos;

    public RunAction openClaw, closeClaw, twistOWrist, retractOArm, liftOArm;
    private final double ARM_RETRACT = 0.5;
    private final double ARM_HIGHER = 0.5;
    private final double WRIST_POS = 0.5;

    public Outake(HardwareMap hardwareMap, clawState clawState) {

        oClaw = hardwareMap.get(ServoImplEx.class, "oClaw");
        this.state = clawState;
        lOutake = hardwareMap.get(ServoImplEx.class, "leftOutake");
        rOutake = hardwareMap.get(ServoImplEx.class, "rightOutake");
        outakeWrist = hardwareMap.get(ServoImplEx.class, "outakeWrist");

        openClaw = new RunAction(this::openClaw);
        closeClaw = new RunAction(this::closeClaw);
        twistOWrist = new RunAction(this::twistOWrist);
        retractOArm = new RunAction(this::retractOArm);
        liftOArm = new RunAction(this::liftOArm);

    }

    public void setPos(double clawPos) {
        oClaw.setPosition(clawPos);
    }

    public void setState(clawState state) {
        if (state == clawState.CLOSED) {
            oClaw.setPosition(clawClosed);
            this.state = clawState.CLOSED;
        } else if (state == clawState.OPEN) {
            oClaw.setPosition(clawOpen);
            this.state = clawState.OPEN;
        }
    }

    public void switchState() {
        if (state == clawState.CLOSED) {
            setState(clawState.OPEN);
        } else if (state == clawState.OPEN) {
            setState(clawState.CLOSED);
        }
    }

    public void openClaw() {
        setState(clawState.OPEN);
    }

    public void closeClaw() {
        setState(clawState.CLOSED);
    }

    public void twistOWrist() {
        outakeWrist.setPosition(WRIST_POS);
    }

    public void retractOArm () {
        lOutake.setPosition(ARM_RETRACT);
        rOutake.setPosition(ARM_RETRACT);
    }

    public void liftOArm() {
        lOutake.setPosition(ARM_HIGHER);
        rOutake.setPosition(ARM_HIGHER);
    }

    public void checkPos() {
        wristPos = outakeWrist.getPosition();
        clawPos = oClaw.getPosition();
    }
/*
    public void init() {
        Actions.runBlocking(closeClaw);
    }

    public void start() {
        Actions.runBlocking(closeClaw);
    }
*/
}
