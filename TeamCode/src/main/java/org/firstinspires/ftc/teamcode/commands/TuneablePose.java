package org.firstinspires.ftc.teamcode.commands;

//import com.pedropathing.

import com.pedropathing.math.Pose;

public class TuneablePose {
    public double x;
    public double y;
    public double heading;

    public TuneablePose(double x, double y, double headingDegrees) {
        this.x = x;
        this.y = y;
        this.heading = headingDegrees;
    }

    public Pose getPose()
    {
        return new Pose(x, y, Math.toRadians(heading));
    }

    public Pose getCorrectPose(boolean red)
    {
        return red ? getRedPose() : getPose();
    }


    public Pose getRedPose() {
        return new Pose(144 - x, y, Math.toRadians(-heading));
    }
}