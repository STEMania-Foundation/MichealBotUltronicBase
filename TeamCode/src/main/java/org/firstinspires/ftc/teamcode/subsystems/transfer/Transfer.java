package org.firstinspires.ftc.teamcode.subsystems.transfer;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.lib.util.KTelemetry;
import org.firstinspires.ftc.teamcode.subsystems.flywheel.Flywheel;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//bleh bleh bleh
//^frfr

public class Transfer extends SubsystemBase {
    private final Servo gate;

    private final KTelemetry telemetry;
    private final MotorEx transfer;

    public Transfer(HardwareMap hwMap, KTelemetry telemetry ) {
        this.telemetry = telemetry;

        transfer = new MotorEx(hwMap, "transfer");
        gate = hwMap.get(Servo.class, "gate");


    }

    @Override
    public void periodic() {
        try {
            telemetry.addLine("Transfer");
        } catch (Exception ignored) {

        }
        if(transfer.getVelocity()>0){
                gate.setPosition(0);

        }else{
            gate.setPosition(1);
        }

    }

    public void setPower(double power) {
        transfer.set(power);
    }

    public static Command setPower(Transfer transfer, DoubleSupplier power) {
        return Commands.run(() -> transfer.setPower(power.getAsDouble()), transfer);
        
    }

    public static Command setPower(Transfer transfer, double power) {
        return setPower(transfer, () -> power);
    }
    public BooleanSupplier transferRunning() {
        return () -> Math.abs(transfer.motor.getPower()) > 0.1;
    }




}
