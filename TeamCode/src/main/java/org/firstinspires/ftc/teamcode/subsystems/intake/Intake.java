package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.lib.util.KTelemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//bleh bleh bleh

public class Intake extends SubsystemBase {

    private final KTelemetry telemetry;

    private final MotorEx intake;
    //private final DigitalChannel transfer_BB, artifact_2_BB;

    private boolean transfer_beam, second_beam, third_beam;

    public Intake(HardwareMap hwMap, KTelemetry telemetry) {
        this.telemetry = telemetry;

        intake = new MotorEx(hwMap, "intake");
                //transfer_BB = hwMap.get(DigitalChannel.class, "firstBB");
        //artifact_2_BB = hwMap.get(DigitalChannel.class, "secondBB");
    }

    @Override
    public void periodic() {
        try {
            //transfer_beam = !transfer_BB.getState();
            //second_beam = !artifact_2_BB.getState();


            telemetry.addLine("Intake");
            //telemetry.addData("Transfer Beam:", transfer_beam);
            //telemetry.addData("Second Beam:", second_beam);
        } catch (Exception ignored) {

        }
    }

    public void setPower(double power) {
        intake.set(-power);
    }

    public static Command setPower(Intake intake, DoubleSupplier power) {
        return Commands.run(() -> intake.setPower(power.getAsDouble()), intake);
    }

    public static Command setPower(Intake intake, double power) {
        return setPower(intake, () -> power);
    }
    public BooleanSupplier intakeRunning() {
        return () -> Math.abs(intake.motor.getPower()) > 0.1;
    }




}
