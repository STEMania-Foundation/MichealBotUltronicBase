package org.firstinspires.ftc.teamcode.lib.ftclib.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

// An OpMode that is has methods similar to WPILib's Robot class
public abstract class TimedRobotOpMode extends OpMode {

    private List<LynxModule> allHubs;

    @Override
    public final void init() {
        Robot.disable();
        CommandScheduler.getInstance().setRobotEnabled(false);
        configureBulkCaching();
        robotInit();
    }
    @Override
    public final void init_loop() {
        clearBulkCaches();
        robotPeriodic();
    }

    @Override
    public final void start() {
        Robot.enable();
        CommandScheduler.getInstance().setRobotEnabled(true);
        enabledInit();
    }

    @Override
    public final void loop() {
        // loop() only runs after PLAY, so assert it here too rather than depending on start()
        // alone having fired.
        CommandScheduler.getInstance().setRobotEnabled(true);
        clearBulkCaches();
        robotPeriodic();
        enabledPeriodic();
    }

    @Override
    public final void stop() {
        disabledInit();
    }

    /**
     * Puts every hub into MANUAL bulk-caching mode. Done here rather than per-opmode so the
     * autonomous opmodes get it too, and so it happens before robotInit() builds hardware.
     */
    private void configureBulkCaching() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     * Clears the caches at the top of every loop, so each hub is read exactly once per loop and
     * everything downstream sees a consistent snapshot.
     */
    private void clearBulkCaches() {
        if (allHubs == null) {
            return;
        }
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

    abstract public void robotInit();

    abstract public void robotPeriodic();

    abstract public void enabledInit();

    abstract public void enabledPeriodic();

    abstract public void disabledInit();
}
