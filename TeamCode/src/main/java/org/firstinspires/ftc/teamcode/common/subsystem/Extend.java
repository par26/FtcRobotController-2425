package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.action.Actions;
import org.firstinspires.ftc.teamcode.common.action.RunAction;
import org.firstinspires.ftc.teamcode.common.utils.RobotConstants;

public class Extend {

    private Telemetry telemetry;

    private ServoImplEx lExtend;
    private ServoImplEx rExtend;

    private double extended;
    private double retracted;

    public RunAction extendEx, retractEx;

    public enum extendState {
        EXTENDED, RETRACTED
    }

    private extendState state;

    public Extend(HardwareMap hardwareMap) {
        lExtend = hardwareMap.get(ServoImplEx.class, "leftExtend");
        rExtend = hardwareMap.get(ServoImplEx.class, "rightExtend");

        lExtend.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rExtend.setPwmRange(new PwmControl.PwmRange(500, 2500));



        extended = RobotConstants.EX_EXTEND;
        retracted = RobotConstants.EX_RETRACT;

        state = extendState.RETRACTED;

        lExtend.setDirection(Servo.Direction.REVERSE);

        extendEx = new RunAction(this::extendExtend);
        retractEx = new RunAction(this::retractExtend);

    }

    public void setExtendState(Extend.extendState state) {
        if(state == extendState.EXTENDED) {
            lExtend.setPosition(extended);
            rExtend.setPosition(extended);
            this.state = extendState.EXTENDED;
        } else if(state == extendState.RETRACTED) {
            lExtend.setPosition(retracted);
            rExtend.setPosition(retracted);
            this.state = extendState.RETRACTED;
        }
    }


    public void switchExtendState() {
        if(this.state == extendState.RETRACTED) {
            setExtendState(extendState.EXTENDED);
        } else {
            setExtendState(extendState.RETRACTED);
        }
    }


    public void retractExtend() {
        setExtendState(extendState.RETRACTED);
    }

    public void extendExtend() {
        setExtendState(extendState.EXTENDED);
    }

    public String getState() {
        return this.state == extendState.EXTENDED ? "Extended" : "Retracted";
    }

    public void init() {
        Actions.runBlocking(retractEx);
    }

    public void start() {
        Actions.runBlocking(retractEx);
    }
}
