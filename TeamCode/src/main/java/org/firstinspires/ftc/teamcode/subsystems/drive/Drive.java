package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.subsystems.drive.TeleConstants.teleOP_Squid_P;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;


import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.controller.SquIDController;
import org.firstinspires.ftc.teamcode.lib.util.KTelemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final Follower drive;
    private final KTelemetry telemetry;

    private SquIDController headingTelePID;


    private double distanceToGoal = 0;

    public Drive(HardwareMap hwMap, KTelemetry telemetry) {
        drive = Constants.create(hwMap);

//        drive.followPath(new Path(new BezierPoint(new Pose())));
        this.telemetry = telemetry;

        headingTelePID = new SquIDController(teleOP_Squid_P);
    }

    @Override
    public void periodic() {
        try {
            headingTelePID.setP(teleOP_Squid_P);

            drive.update();

            /*
            telemetry.addData("Pose: X", drive.pose().x());
            telemetry.addData("Pose: Y", drive.pose().y());
            telemetry.addData("Pose: Heading", drive.pose().heading());
            telemetry.addData("Pose: Heading Deg", Math.toDegrees(drive.pose().heading()));


             */
            Velocity velocity = drive.velocity();
            /*
            telemetry.addData("Vel: X", velocity.vx);
            telemetry.addData("Vel: Y", velocity.vy);
            telemetry.addData("Vel: Heading", velocity.omega);
            telemetry.addData("Vel: Speed", Math.hypot(velocity.vx, velocity.vy));

             */

        } catch (Exception e) {
            telemetry.addData("Drive ERROR", e.getClass().getSimpleName() + ": " + e.getMessage());
        }
    }

    public void setPose(Pose pose) {
        drive.setPose(pose);
    }

    public void startTeleopDrive() {
        drive.manual();
    }

    public void drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, BooleanSupplier isFieldCentric) {
        if (isFieldCentric.getAsBoolean()) {
            DrivePowers powers = ManualDrive.fieldCentric(xSupplier.getAsDouble() , ySupplier.getAsDouble(), rotationSupplier.getAsDouble(),drive.pose().heading());
            drive.manual(powers);

        } else {
            drive.manual(xSupplier.getAsDouble(), ySupplier.getAsDouble(),rotationSupplier.getAsDouble());
        }
    }

    public Pose getPose() {
        return drive.pose();
    }

    public Vector getVelocity() {
        return drive.velocity().toVector();
    }

    public boolean isStill() {
        return Math.hypot(drive.velocity().vx,drive.velocity().vy) < 0.1 && Math.abs(drive.velocity().omega) < 0.1;
    }

    public double getXVelocity() {
        return drive.velocity().vx;
    }

    public double getYVelocity() {
        return drive.velocity().vy;

    }


    public double getHeadingVelocity() {
        return drive.velocity().omega;
    }

    public Pose getDesiredPose() {
        return drive.closestPose();
    }

    public double getPathT() {
        return drive.completion();
    }

    public void followPath(Path path) {
        drive.follow(path);
    }

    public boolean isFinished() {
        return !drive.isBusy();
    }

    public boolean headingIsFinished() {
        // Angle.error is normalizeSigned(b - a), so it is signed over (-pi, pi]. Without the
        // abs() every negative error reads as finished.
        return Math.abs(Angle.error(drive.pose().heading(), drive.closestPose().heading())) < 0.015; // Adjust this threshold as needed
    }

    public double speed(){
        return Math.abs(drive.velocity().vx)+Math.abs(drive.velocity().vy);

    }

    public static Command followPath(Drive drive, Pose startPose, Path path) {
        return Commands.runOnce(() -> drive.setPose(startPose), drive)
                .andThen(Commands.runOnce(() -> drive.followPath(path), drive));
    }

    public static Command followPath(Drive drive, Path path) {
        return Commands.runOnce(() -> drive.followPath(path), drive).andThen(Commands.waitUntil(drive::isFinished));
    }

    public static Command followPath(Drive drive, Supplier<Path> path) {
        return Commands.runOnce(() -> drive.followPath(path.get()), drive).andThen(Commands.waitUntil(drive::isFinished));
    }
    public Follower getFollower() {
        return drive;
    }

    public void resetPinpoint() throws InterruptedException {
        drive.localizer.setHeading(0.0);
    }
}