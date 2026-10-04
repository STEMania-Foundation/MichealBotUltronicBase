package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveCommands {

    private DriveCommands() {}

    public BooleanSupplier isStationary(Drive drive) {
        return drive::isStill;
    }


    public static Command drive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, BooleanSupplier isFieldCentric) {
        return Commands.runOnce(drive::startTeleopDrive, drive).andThen(
                Commands.run(
                        () -> drive.drive(
                                xSupplier,
                                ySupplier,
                                rotationSupplier,isFieldCentric),
                        drive));

    }
    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */

    // Drive to point — precise arrival, holds end, finishes when within tolerance
    public static Command driveToPose(Drive drive, Supplier<Pose> pose) {
        return Drive.followPath(
                drive,
                () -> {
                    Pose current = drive.getPose();
                    Pose target = pose.get();
                    return Paths.line(current,target).linear(current,target);
                });
    }

    public static Command driveToPose(Drive drive, Supplier<Pose> pose, Command command, DoubleSupplier commandActivationPoint) {
        return Commands.parallel(
                driveToPose(drive, pose),
                Commands.sequence(
                        Commands.waitUntil(() -> drive.getPathT() > commandActivationPoint.getAsDouble()),
                        command
                )
        );
    }


    public static Command driveNearPoint(Drive drive, Supplier<Pose> pose, double radiusInches) {
        return Commands.runOnce(() -> {
            Pose current = drive.getPose();
            Pose target = pose.get();
            drive.followPath(Paths.line(current,target).linear(current,target));
        }, drive).andThen(
                Commands.waitUntil(() -> {
                    Pose current = drive.getPose();
                    Pose target = pose.get();
                    double dx = current.x() - target.x();
                    double dy = current.y() - target.y();
                    return (dx * dx + dy * dy) <= radiusInches * radiusInches;
                })
        );
    }

    public static Command driveTangential(Drive drive, boolean reversed, boolean red, TuneablePose endPose, TuneablePose... controlPoints) {
        return Commands.runOnce(() -> {
            ArrayList<Pose> poseList = new ArrayList<>();

            poseList.add(drive.getPose());
            for (TuneablePose cp : controlPoints) {
                poseList.add(cp.getCorrectPose(red));
            }

            poseList.add(endPose.getCorrectPose(red));
            Path path = Paths.curve(poseList.toArray(new Pose[0]));

            if (reversed) {
                path = path.reverseTangent();
            } else {
                path = path.tangent();
            }

            drive.followPath(path);
        }, drive).andThen(
                Commands.waitUntil(drive::isFinished)
        );
    }

    public static Command driveTangentialNear(Drive drive, boolean reversed, boolean red, double radiusInches, TuneablePose endPose, TuneablePose... controlPoints) {
        return Commands.runOnce(() -> {
            List<Pose> poseList = new ArrayList<>();
            poseList.add(drive.getPose());

            for (TuneablePose cp : controlPoints) {
                poseList.add(cp.getCorrectPose(red));
            }

            poseList.add(endPose.getCorrectPose(red));
            Path path = Paths.curve(poseList.toArray(new Pose [0]));
            if (reversed) {
                path = path.reverseTangent();
            } else {
                path = path.tangent();
            }

            drive.followPath(path);

        }, drive).andThen(
                Commands.waitUntil(() -> {
                    Pose current = drive.getPose();
                    Pose target = endPose.getCorrectPose(red);
                    double dx = current.x() - target.x();
                    double dy = current.y() - target.y();
                    return (dx * dx + dy * dy) <= radiusInches * radiusInches;
                }));
    }

    @SafeVarargs
    public static Command driveLinear(Drive drive, Supplier<Pose> endPose, DoubleSupplier endHeading, Supplier<Pose>... controlPoints) {
        return Commands.runOnce(() -> {
            Pose startPose = drive.getPose();
            List<Pose> poses = new ArrayList<>();
            poses.add(startPose);

            for (Supplier<Pose> cp : controlPoints) {
                poses.add(cp.get());
            }
            poses.add(endPose.get());
            Path path;
            if (poses.size() >2) {
                path = Paths.curve(poses.toArray(new Pose[0]));
            } else {
                path = Paths.line(startPose,endPose.get());
            }
            path = path.linear(startPose.heading(), endHeading.getAsDouble());
            drive.followPath(path);
        }, drive).andThen(
                Commands.waitUntil(drive::isFinished)
        );
    }


    public static Command setPose(Drive drive, Supplier<Pose> pose) {
        return Commands.runOnce(() -> drive.setPose(pose.get()));
    }

    public static Command setHeading(Drive drive, DoubleSupplier heading) {
        return Commands.runOnce(() -> drive.setPose(new Pose(drive.getPose().x(), drive.getPose().y(), heading.getAsDouble())), drive);
    }



//    public static Command turn(Drive drive, DoubleSupplier angle) {
//        return Drive.followPath(
//                drive,
//                () -> drive.getPathBuilder()
//                        .addPath(new Path(new BezierPoint(drive.getDesiredPose().getPose())))
//                        .setConstantHeadingInterpolation(drive.getDesiredPose().getPose().getHeading() + Math.toRadians(angle.getAsDouble()))
//                        .build()).until(drive::headingIsFinished).andThen(Commands.waitSeconds(0.1));
//    }
    public static Command turn(Drive drive, DoubleSupplier angle) {
        return Commands.runOnce(() -> {
            Pose current = drive.getPose();
            double targetHeading = current.heading() + Math.toRadians(angle.getAsDouble());
            drive.getFollower().hold(new Pose(current.x(), current.y(), targetHeading));
        }).andThen(Commands.waitUntil(drive::isFinished));
    }

    public static Command setPoseStorage(Drive drive, Supplier<Pose> pose) {
        return Commands.runOnce(() -> {
            PoseStorage.currentPose = pose.get();
        }, drive);
    }

    public static double signSquare(double num) {
        return num * num * Math.signum(num);
    }

}

