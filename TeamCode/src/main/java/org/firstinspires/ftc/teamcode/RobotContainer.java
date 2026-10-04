package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.commands.TuneablePose;
import org.firstinspires.ftc.teamcode.commands.auto.closeAutos;
import org.firstinspires.ftc.teamcode.lib.util.KTelemetry;
import org.firstinspires.ftc.teamcode.lib.wpilib.CommandGamepad;
import org.firstinspires.ftc.teamcode.lib.wpilib.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.opmodes.OpModeConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.flywheel.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;


import java.util.Iterator;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class RobotContainer {
    private final KTelemetry telemetry;
    private final Drive drive;


    private final Intake intake;
    private final Transfer transfer;
    private final Flywheel flywheel;

    private final CommandGamepad driverController;

    private boolean isRed;

    private boolean telemetryActive = true;

    public static DoubleSupplier voltage;

    // how often to read the battery
    private static final double VOLTAGE_PERIOD_MS = 100.0;

    private final VoltageSensor voltageSensor;
    private double cachedVoltage = 12.0;
    private double lastVoltageReadMs = Double.NEGATIVE_INFINITY;



    public static TuneablePose redStartPose = new TuneablePose(6.4,9.8,180);
    public static TuneablePose blueStartPose = new TuneablePose(136, 9.8,0);

    //public static TuneablePose blueStartPose = new TuneablePose(42.5, 10.6, 180);

    public RobotContainer(HardwareMap hwMap, KTelemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, OpModeConstants autoNum, boolean red) {
        this.telemetry = telemetry;

        drive = new Drive(hwMap, telemetry);

        intake = new Intake(hwMap, telemetry);
        transfer = new Transfer(hwMap, telemetry);
        flywheel = new Flywheel(hwMap, telemetry);

        driverController = new CommandGamepad(gamepad1);

        Iterator<VoltageSensor> sensors = hwMap.voltageSensor.iterator();
        voltageSensor = sensors.hasNext() ? sensors.next() : null;
        voltage = this::readVoltage;

        // This needs to be before all the default commands lmao
        this.isRed = red;

        if (autoNum == OpModeConstants.TELEOP) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            getAutoCommand(autoNum);
        }

    }

    public void periodic() {
        telemetry.addData("Telemetry Active", telemetryActive);
        telemetry.addData("Battery Voltage", readVoltage());
    }
    private double readVoltage() {
        if (voltageSensor == null) {
            return 12.0;
        }
        double now = System.currentTimeMillis();
        if (now - lastVoltageReadMs >= VOLTAGE_PERIOD_MS) {
            cachedVoltage = voltageSensor.getVoltage();
            lastVoltageReadMs = now;
        }
        return cachedVoltage;
    }


    public void setDefaultCommands(){
        intake.setDefaultCommand(Commands.run(() -> intake.setPower(0.0), intake));
        transfer.setDefaultCommand(Commands.run(() -> transfer.setPower(0.0), transfer));

        if (isRed) {
            drive.setDefaultCommand(
                    DriveCommands.drive(drive,
                            () -> (driverController.getLeftY()*-1),
                            () -> (driverController.getLeftX()*-1),
                            () -> (-(driverController.getRightX())*0.9),
                            //() -> ((-(driverController.getRightX())/voltage.getAsDouble()*12.6)*0.9),
                            () -> true
                    )
            );
        } else {
            drive.setDefaultCommand(
                    DriveCommands.drive(drive,
                            () -> (driverController.getLeftY()*1),
                            () -> (driverController.getLeftX()*1),
                            () -> (-(driverController.getRightX())*0.9),
                            //() -> ((-(driverController.getRightX())/voltage.getAsDouble()*12.6)*0.9),
                            () -> true
                    )
            );
        }
    }



    public void configureButtonBindings() {

        driverController.start().onTrue(
                Commands.either(
                        DriveCommands.setPose(drive, ()-> redStartPose.getPose()),
                        DriveCommands.setPose(drive, ()-> blueStartPose.getPose()),
                        ()->isRed)
        );

        driverController.rightBumper().whileTrue(Commands.run(() -> intake.setPower(1.0), intake));
        driverController.leftBumper().whileTrue(Commands.run(() -> transfer.setPower(1.0), transfer));

        driverController.a().onTrue(flywheel.setTarget(() -> FlywheelConstants.shootVelocity));
        driverController.b().onTrue(flywheel.setTarget(() -> 0.0));
    }

    public Command getAutoCommand(OpModeConstants auto) {
        return switch (auto) {
            case BLUE_AUTO -> closeAutos.Close_21_Auto(drive, intake, false);
            case RED_Auto -> closeAutos.Close_21_Auto(drive, intake, true);

            default -> Commands.none();
        };
    }

    public Follower getFollower() {
        return drive.getFollower();
    }

    public boolean getTelemetryActive() {
        return telemetryActive;
    }
}