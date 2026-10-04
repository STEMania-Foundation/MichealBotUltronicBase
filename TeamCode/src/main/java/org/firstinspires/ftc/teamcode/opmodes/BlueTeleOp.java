package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.util.KTelemetry;
import org.firstinspires.ftc.teamcode.lib.ftclib.opmode.CommandOpMode;

import edu.wpi.first.wpilibj.Timer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BlueTeleOp", group = "TeleOp")
public class BlueTeleOp extends CommandOpMode {

    private RobotContainer robotContainer;
    private KTelemetry robotTelemetry;
    private Timer timer = new Timer();

    private double previousTime;

    @Override
    public void robotInit() {
        robotTelemetry = new KTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
        robotContainer = new RobotContainer(hardwareMap, robotTelemetry, gamepad1, gamepad2, OpModeConstants.TELEOP, false); //Uses heavily modified untested hardware
        timer.start();
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        robotContainer.periodic();

        robotTelemetry.addData("Loop Hz", 1.0 / (timer.get() - previousTime));
        robotTelemetry.update(robotContainer.getTelemetryActive());
        previousTime = timer.get();

    }
}