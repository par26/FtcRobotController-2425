package org.firstinspires.ftc.teamcode.common.action;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.Timer;
import java.util.TimerTask;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SleepAction implements Action {
    private final double dt;
    private boolean isFinished = false;
    private ElapsedTime timer;

    public SleepAction(double dt) {
        this.dt = dt;
        this.timer = new ElapsedTime();
    }

    @Override
    public boolean run(TelemetryPacket p) {
        if (!isFinished) {
            if (timer.seconds() >= dt) {
                isFinished = true;
                return false;
            }
            return true;
        }
        return false;
    }
}