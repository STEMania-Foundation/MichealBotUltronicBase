package org.firstinspires.ftc.teamcode.lib.util;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayList;
import java.util.List;

public class FieldDrawing {
    public static final double ROBOT_RADIUS = 18.0 / 2.0;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final int MAX_HISTORY = 100;
    private static final List<Pose> poseHistory = new ArrayList<>();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.1
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.0
    );

    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower == null) return;

        if (follower.currentPath() != null) {
            drawPath(follower.currentPath(), robotLook);

            Pose targetPose = follower.closestPose();
            if (targetPose != null) {
                drawRobot(targetPose, robotLook);
            }
        }

        Pose currentPose = follower.pose();
        recordPose(currentPose);
        drawPoseHistory(historyLook);
        drawRobot(currentPose, historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.x(), pose.y());
        panelsField.circle(ROBOT_RADIUS);

        double cos = Math.cos(pose.heading()) * ROBOT_RADIUS;
        double sin = Math.sin(pose.heading()) * ROBOT_RADIUS;
        panelsField.moveCursor(pose.x() + cos / 2.0, pose.y() + sin / 2.0);
        panelsField.line(pose.x() + cos, pose.y() + sin);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        if (path == null) return;

        panelsField.setStyle(style);
        Pose start = path.get(0.0);
        panelsField.moveCursor(start.x(), start.y());

        int resolution = 25;
        for (int i = 1; i <= resolution; i++) {
            double t = (double) i / resolution;
            Pose p = path.get(t);
            if (!Double.isNaN(p.x()) && !Double.isNaN(p.y())) {
                panelsField.line(p.x(), p.y());
            }
        }
    }

    private static void recordPose(Pose pose) {
        if (pose == null) return;
        poseHistory.add(pose);
        if (poseHistory.size() > MAX_HISTORY) {
            poseHistory.remove(0);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param style the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(Style style) {
        if (poseHistory.size() < 2) return;

        panelsField.setStyle(style);
        Pose first = poseHistory.get(0);
        panelsField.moveCursor(first.x(), first.y());

        for (int i = 1; i < poseHistory.size(); i++) {
            Pose p = poseHistory.get(i);
            panelsField.line(p.x(), p.y());
        }
    }

    /**
     * Clears the recorded pose history trail.
     */
    public static void clearHistory() {
        poseHistory.clear();
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}
