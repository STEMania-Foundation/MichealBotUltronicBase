package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.telemetry.SelectScope;
import com.pedropathing.telemetry.Selector;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.ftclib.opmode.CommandOpMode;
import org.firstinspires.ftc.teamcode.lib.util.FieldDrawing;
import org.firstinspires.ftc.teamcode.lib.util.KTelemetry;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class SelectableAutoOpMode extends CommandOpMode {

    /** A menu entry: given the container, produce the command to run. */
    @FunctionalInterface
    public interface Routine extends Function<RobotContainer, Command> {}

    private static final String[] MESSAGE = {
            "D-pad up/down to move the cursor.",
            "Right bumper to select.",
            "Left bumper to go back."
    };

    private final Selector<Routine> selector;
    private Routine selected;
    private boolean started;

    protected KTelemetry robotTelemetry;
    protected RobotContainer robotContainer;

    private final Timer timer = new Timer();
    private double previousTime;

    protected SelectableAutoOpMode(String name, Consumer<SelectScope<Routine>> routines) {
        selector = Selector.create(name, routines, MESSAGE);
        selector.onSelect(routine -> {
            selected = routine;
            onSelect(routine);
        });
    }

    /** Build the alliance-specific container. Called once, at init. */
    protected abstract RobotContainer createRobotContainer();

    protected void onSelect(Routine routine) {}
    protected void onLog(List<String> lines) {}

    @Override
    public void robotInit() {
        CommandScheduler.getInstance().reset();

        robotTelemetry = new KTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
        FieldDrawing.init();

        robotContainer = createRobotContainer();
        timer.start();
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();                 // scheduler actually runs now

        if (!started) updateSelector();

        robotContainer.periodic();

        robotTelemetry.addData("Loop Hz", 1.0 / (timer.get() - previousTime));
        previousTime = timer.get();

        FieldDrawing.drawDebug(robotContainer.getFollower());
        robotTelemetry.update(robotContainer.getTelemetryActive());
    }

    private void updateSelector() {
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed())
            selector.decrementSelected();
        else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed())
            selector.incrementSelected();
        else if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed())
            selector.select();
        else if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed())
            selector.goBack();

        List<String> lines = selector.getLines();
        for (String line : lines) telemetry.addLine(line);
        if (selected == null) telemetry.addLine(">>> NO AUTO SELECTED <<<");
        onLog(lines);
    }

    @Override
    public void enabledInit() {
        started = true;

        if (selected == null) {
            robotTelemetry.addData("ERROR", "Started with no auto selected - doing nothing");
            return;
        }
        selected.apply(robotContainer).schedule();
    }
}