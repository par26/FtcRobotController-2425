package org.firstinspires.ftc.teamcode.common.action;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class OrAction implements Action {
    private List<Action> actions;
    private boolean isFinished;

    public OrAction(List<Action> actions) {
        this.actions = new ArrayList<>(actions);
    }

    public OrAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean run(TelemetryPacket p) {
        if (isFinished) {
            return true; // If already finished, return immediately
        }

        // Run all actions and check if any of them finishes
        for (Action action : actions) {
            if (!action.run(p)) {
                // If any action finishes, mark this action as finished
                isFinished = true;
                break;
            }
        }

        // If any action finished, stop the entire race
        return isFinished;

    }

    @Override
    public void preview(Canvas fieldOverlay) {
        for (Action a : actions) {
            a.preview(fieldOverlay);
        }
    }
}