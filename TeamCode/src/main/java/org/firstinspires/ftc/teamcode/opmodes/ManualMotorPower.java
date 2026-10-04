package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@Configurable
@TeleOp(name="Manual Motor Power", group = "testing")
@Disabled
public class ManualMotorPower extends LinearOpMode {

    public static double br, bl, fl, fr;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;

    public List<LynxModule> allHubs;
    private double loopTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Names match pedro/Constants.java MecanumConfig; the hub config is case-sensitive.
        backRight = hardwareMap.get(DcMotorEx.class, "BR");
        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");

        waitForStart();

        loopTime = System.nanoTime();

        while (opModeIsActive()) {
            clearBulkCache();

            backRight.setPower(-br);
            backLeft.setPower(bl);
            frontRight.setPower(-fr);
            frontLeft.setPower(fl);

            panelsTelemetry.addData("hz", getHz());
            panelsTelemetry.update();
        }

    }

    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            if (!(hub.getDeviceName().equals("Servo Hub 3"))) {
                hub.clearBulkCache();
            }
        }
    }

    public double getHz(){
        double loop = System.nanoTime();
        double hz = 1000000000 / (loop - loopTime);
        loopTime = loop;
        return hz;
    }
}