package org.firstinspires.ftc.teamcode.common.action;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.Timer;
import java.util.TimerTask;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SleepAction implements Action {
    private final double dt;
    private boolean isFinished;
    private final ElapsedTime timer;
    private boolean isTimerInitialized; // Flag to track if the timer has been reset

    public SleepAction(double dt) {
        if (dt < 0) {
            throw new IllegalArgumentException("Sleep time cannot be negative");
        }
        this.dt = dt;
        this.timer = new ElapsedTime();
        this.isFinished = (dt == 0); // Immediately finish if dt is 0
        this.isTimerInitialized = false; // Timer has not been reset yet
    }

    @Override
    public synchronized boolean run(TelemetryPacket p) {
        // Reset the timer the first time run() is called
        if (!isTimerInitialized) {
            timer.reset();
            isTimerInitialized = true;
        }

        if (!isFinished) {
            p.put("Sleep Time Remaining", dt - timer.seconds());
            if (timer.seconds() >= dt) {
                isFinished = true;
                return false; // Action is finished
            }
            return true; // Action is still running
        }
        return false; // Action is finished
    }

    public void reset() {
        timer.reset();
        isFinished = false;
        isTimerInitialized = false; // Reset the flag to allow timer reset on next run()
    }
}