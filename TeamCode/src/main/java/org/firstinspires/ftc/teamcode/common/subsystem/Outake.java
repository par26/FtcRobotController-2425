package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.action.Actions;
import org.firstinspires.ftc.teamcode.common.action.RunAction;
import org.firstinspires.ftc.teamcode.common.action.SequentialAction;
import org.firstinspires.ftc.teamcode.common.utils.RobotConstants;

public class Outake {

    public enum clawState {
        CLOSED, OPEN
    }

    public enum armState {
        TRANSFER, BUCKET, SPECIMEN
    }

    private clawState claw;
    private ServoImplEx oClaw;
    private ServoImplEx lOutake;
    private ServoImplEx rOutake;


    private double clawOpen, clawClose;

    double clawPos;

    public RunAction openClaw, closeClaw, toTransfer, toBucket, toSpecimen, liftOArm;
    private double ARM_RETRACT = 0.5;
    private double ARM_SPECIMEN;
    private double ARM_BUCKET;

    public Outake(HardwareMap hardwareMap) {


        ARM_RETRACT = RobotConstants.OUTAKE_ARM_LOWER;
        ARM_BUCKET= RobotConstants.OUTAKE_ARM_BUCKET;
        ARM_SPECIMEN = RobotConstants.OUTAKE_ARM_SPECIMEN;

        clawOpen = RobotConstants.CLAW_OPEN;
        clawClose = RobotConstants.CLAW_CLOSE;


        oClaw = hardwareMap.get(ServoImplEx.class, "oClaw");
        lOutake = hardwareMap.get(ServoImplEx.class, "leftOutake");
        rOutake = hardwareMap.get(ServoImplEx.class, "rightOutake");


        this.claw = clawState.CLOSED;
        lOutake.setDirection(Servo.Direction.REVERSE);


        openClaw = new RunAction(this::openClaw);
        closeClaw = new RunAction(this::closeClaw);

        toTransfer = new RunAction(this::toTransfer);
        toBucket = new RunAction(this::toBucket);
        toSpecimen = new RunAction(this::toSpeicmen);
        liftOArm = new RunAction(this::toBucket);
    }

    public void setClawState(clawState state) {
        if (state == clawState.CLOSED) {
            oClaw.setPosition(clawClose);
            this.claw = clawState.CLOSED;
        } else if (state == clawState.OPEN) {
            oClaw.setPosition(clawOpen);
            this.claw = clawState.OPEN;
        }
    }

    public clawState getClawState() {
        return claw;
    }


    public void switchClawState() {
        if (claw == clawState.CLOSED) {
            setClawState(clawState.OPEN);
        } else if (claw == clawState.OPEN) {
            setClawState(clawState.CLOSED);
        }
    }

    public void openClaw() {
        setClawState(clawState.OPEN);
    }

    public void closeClaw() {
        setClawState(clawState.CLOSED);
    }

    public void toTransfer () {
        lOutake.setPosition(ARM_RETRACT);
        rOutake.setPosition(ARM_RETRACT);
        Actions.runBlocking(new SequentialAction(openClaw));
    }

    public void toBucket() {
        Actions.runBlocking(new SequentialAction(closeClaw));
        lOutake.setPosition(ARM_BUCKET);
        rOutake.setPosition(ARM_BUCKET);
    }

    public void toSpeicmen() {
        Actions.runBlocking(new SequentialAction(closeClaw));
        lOutake.setPosition(ARM_SPECIMEN);
        rOutake.setPosition(ARM_SPECIMEN);
    }

    public void checkPos() {
        clawPos = oClaw.getPosition();
    }

    public void init() {
        Actions.runBlocking(closeClaw);
    }

    public void start() {
        Actions.runBlocking(closeClaw);
        Actions.runBlocking(toTransfer);


    }

}
