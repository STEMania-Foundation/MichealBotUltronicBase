package org.firstinspires.ftc.teamcode.subsystems.turret;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.util.KTelemetry;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Turret extends SubsystemBase{
    private final KTelemetry telemetry;
    private final  Servo LServo;
    private final Servo RServo;
    public Turret(HardwareMap hwMap, KTelemetry telemetry) {
        this.telemetry = telemetry;
        LServo=hwMap.get(Servo.class, "turretA");
        RServo=hwMap.get(Servo.class, "turretB");

    }
    @Override
    public void periodic() {
        try {
            telemetry.addLine("Turret");
            telemetry.addData("LServo", LServo.getPosition());
            telemetry.addData("RServo", RServo.getPosition());
        } catch (Exception ignored) {

        }
    }
    public void setPos(double pos){
        LServo.setPosition(pos);
        RServo.setPosition(1-pos);
        telemetry.addData("LServo", LServo.getPosition());
        telemetry.addData("RServo", RServo.getPosition());
    }
    public static Command setPos(Turret turret, Double pos) {
        return Commands.run(() -> turret.setPos(pos));
    }
    public static Command setPos(Turret turret, DoubleSupplier pos) {
        return Commands.run(() -> turret.setPos(pos.getAsDouble()));
    }
}
