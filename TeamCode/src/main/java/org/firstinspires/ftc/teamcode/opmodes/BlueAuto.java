package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.ftclib.opmode.CommandOpMode;
import org.firstinspires.ftc.teamcode.lib.util.FieldDrawing;
import org.firstinspires.ftc.teamcode.lib.util.KTelemetry;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue_Auto", group = "Auto")
public class BlueAuto extends CommandOpMode {
    private RobotContainer robotContainer;
    private KTelemetry robotTelemetry;
    private Timer timer = new Timer();

    private double previousTime;

    @Override
    public void robotInit() {
        robotTelemetry = new KTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
        robotContainer = new RobotContainer(hardwareMap, robotTelemetry, gamepad1, gamepad2, OpModeConstants.BLUE_AUTO, false); //Uses heavily modified untested hardware
        FieldDrawing.init();
        timer.start();
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        robotContainer.periodic();
        robotTelemetry.addData("Loop Hz", 1.0 / (timer.get() - previousTime));
        robotTelemetry.update(robotContainer.getTelemetryActive());
        FieldDrawing.drawDebug(robotContainer.getFollower());
        previousTime = timer.get();
    }

    @Override
    public void enabledInit() {
        robotContainer.getAutoCommand(OpModeConstants.BLUE_AUTO).schedule();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().reset();
    }
}