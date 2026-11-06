package org.firstinspires.ftc.teamcode.subsystems.turret;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;

import org.firstinspires.ftc.teamcode.lib.util.KTelemetry;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
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
        LServo.setDirection(Direction.FORWARD);
        RServo.setDirection(Direction.REVERSE);

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
    public void reset(){
        TurretConstants.aPos=0.5;
        TurretConstants.bPos=0.5;
        LServo.setPosition(TurretConstants.aPos);
        RServo.setPosition(TurretConstants.bPos);
    }
    public static Command reset(Turret turret){
        return Commands.run(()->turret.reset());
    }
    public void setPos(double pos){
        TurretConstants.aPos=pos;
        TurretConstants.bPos=1-pos;
        LServo.setPosition(TurretConstants.aPos);
        RServo.setPosition(TurretConstants.bPos);
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
