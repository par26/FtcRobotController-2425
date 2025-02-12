package org.firstinspires.ftc.teamcode.common.pedroPathing;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.action.Action;

public class HoldPointAction implements Action {

    private final Follower m_follower;
    private final BezierPoint m_holdPoint;
    private final double m_heading;
    private final double holdTime; // Duration to hold the point in milliseconds
    private final ElapsedTime timer;
    private boolean started = false;
    private boolean isFinished = false;

    public HoldPointAction(Follower follower, BezierPoint holdPoint, double heading, double holdTime) {
        if (holdTime < 0) {
            throw new IllegalArgumentException("Hold time cannot be negative");
        }
        this.m_follower = follower;
        this.m_holdPoint = holdPoint;
        this.m_heading = heading;
        this.holdTime = holdTime;
        this.timer = new ElapsedTime();
        timer.reset();
    }

    public HoldPointAction(Follower follower, Point holdPoint, double heading, double holdTime) {
        this(follower, new BezierPoint(holdPoint), heading, holdTime);
    }

    public HoldPointAction(Follower follower, Pose holdPose, double holdTime) {
        this(follower, new BezierPoint(new Point(holdPose)), holdPose.getHeading(), holdTime);
    }

    @Override
    public boolean run(TelemetryPacket packet) {


        m_follower.update();

        // Check if the hold duration has elapsed
        if (timer.milliseconds() >= holdTime) {
            m_follower.breakFollowing(); // Stop holding
            isFinished = true;
        }

        return !isFinished;
    }
}
