package org.firstinspires.ftc.teamcode.common.action;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class NullAction implements Action {
    @Override
    public boolean run(TelemetryPacket p) {
        return false;
    }
}