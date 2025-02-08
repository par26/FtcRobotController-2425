package org.firstinspires.ftc.teamcode.common.pedroPathing;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.common.action.Action;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.follower.*;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Drawing;

public class FollowPathAction implements Action {

    private final Follower m_follower;
    private Path m_path;
    private PathChain m_pathChain;
    private boolean holdEnd = true;
    private boolean started = false;

    private double maxPower = .65; // Default max power
    private double completionThreshold = 0.99; // Default completion threshold

    public FollowPathAction(Follower follower, Path path, boolean holdEnd) {
        this.m_follower = follower;
        this.m_path = path;
        this.holdEnd = holdEnd;
    }

    public FollowPathAction(Follower follower, Path path) {
        this.m_follower = follower;
        this.m_path = path;
    }

    public FollowPathAction(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.m_follower = follower;
        this.m_pathChain = pathChain;
        this.holdEnd = holdEnd;
    }

    public FollowPathAction(Follower follower, PathChain pathChain) {
        this.m_follower = follower;
        this.m_pathChain = pathChain;
    }

    /**
     * Sets the follower's maximum power.
     * @param power Between 0 and 1
     */
    public void setMaxPower(double power) {
        this.maxPower = power;
        m_follower.setMaxPower(power); // Adjust follower's max power
    }

    /**
     * Checks if the path following is finished.
     * @return True if the path is complete, false otherwise
     */
    public boolean isFinished() {
        if (m_pathChain != null) {
            // For PathChain, check if the current path is the last one and the robot is near the endpoint
            return m_follower.getCurrentPathNumber() == m_pathChain.size() - 1 &&
                    Math.abs(m_follower.headingError) < 0.1 &&
                    m_follower.getCurrentTValue() >= this.completionThreshold;
        } else if (m_path != null) {
            // For a single Path, check completion threshold and heading error
            return Math.abs(m_follower.headingError) < 0.1 &&
                    m_follower.getCurrentTValue() >= this.completionThreshold;
        }
        return false;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!started) {
            if (m_path != null) {
                m_follower.followPath(m_path, holdEnd);
            } else if (m_pathChain != null) {
                m_follower.followPath(m_pathChain, holdEnd);
            }
            m_follower.setMaxPower(maxPower); // Set max power at the start
            started = true;
        }

        m_follower.update();
        Drawing.drawDebug(m_follower);

        // Automatically return isFinished() to control the action lifecycle
        return !isFinished();
    }
}
