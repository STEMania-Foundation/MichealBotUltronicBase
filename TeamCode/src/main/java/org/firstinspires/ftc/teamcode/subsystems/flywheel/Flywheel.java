package org.firstinspires.ftc.teamcode.subsystems.flywheel;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.lib.util.KTelemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

    private final KTelemetry telemetry;

    private final MotorEx leftMotor;
    private final MotorEx rightMotor;

    private final PIDController leftPID;
    private final PIDController rightPID;


    private double leftVelocity = 0;
    private double rightVelocity = 0;
//    private double targetVelocity = 0;

    private boolean run = false;



    public Flywheel(HardwareMap hwMap, KTelemetry telemetry) {
        this.telemetry = telemetry;

        leftMotor = new MotorEx(hwMap, "LFLY");
        leftMotor.setInverted(false);

        rightMotor = new MotorEx(hwMap, "RFLY");
        rightMotor.setInverted(true);

        leftPID = new PIDController(FlywheelConstants.kp, FlywheelConstants.ki, FlywheelConstants.kd);
        rightPID = new PIDController(FlywheelConstants.kp, FlywheelConstants.ki, FlywheelConstants.kd);

        leftPID.setTolerance(5);
        rightPID.setTolerance(5);
    }

    @Override
    public void periodic() {
        leftPID.setPID(FlywheelConstants.kp, FlywheelConstants.ki, FlywheelConstants.kd);
        rightPID.setPID(FlywheelConstants.kp, FlywheelConstants.ki, FlywheelConstants.kd);

        leftMotor.getCurrentPosition();
        rightMotor.getCurrentPosition();

        leftVelocity = leftMotor.getCorrectedVelocity();
        rightVelocity = rightMotor.getCorrectedVelocity();
        updatePowerPID(FlywheelConstants.target);

        telemetry.addLine("Flywheel");
        telemetry.addData("LeftVel", ((leftVelocity/28.0)*60.0)/2.6);
        telemetry.addData("RightVel", ((rightVelocity/28.0)*60.0)/2.6);
        telemetry.addData("Flywheel Left Power", leftMotor.motor.getPower());
        telemetry.addData("Flywheel Right Power", rightMotor.motor.getPower());
    }

    public void updatePowerPID(double velocity) {
        telemetry.addData("Flywheel Desired Velocity", velocity);
        double calculatedtarget = velocity * (28.0/60.0) * 2.6;
        double leftpwr = leftPID.calculate(leftVelocity,calculatedtarget);
        double ff = (FlywheelConstants.ks + FlywheelConstants.kv*calculatedtarget + FlywheelConstants.kf);

        double rightpwr = rightPID.calculate(rightVelocity,calculatedtarget);

        leftMotor.set(leftpwr+ff);
        rightMotor.set(rightpwr+ff);

    }

    public void setVelocity(double velocity) {
        FlywheelConstants.target = velocity;
    }

    public void run(boolean start) {
        run = start;
    }
    
    public Command setTarget(DoubleSupplier velocity) {
        return Commands.runOnce(() -> setVelocity(velocity.getAsDouble()), this);
    }



    public BooleanSupplier flywheelRunning() {
        return () -> Math.abs(leftMotor.motor.getPower()) > 0.1;
    }

}
