package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.localizers.DriveEncoderConfig;
import org.firstinspires.ftc.teamcode.pedro.procedures.OctoQuadTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;

public class Constants {
    // Placeholder values - run the "OctoQuad Tuner" procedure in Tuning.java and paste its
    // generated block over this one. Every var below is ConfigVar.required(); leaving one
    // unset throws the moment OctoQuadLocalizer reads it.
    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> { // TODO: REPLACE WITH REAL CONFIG
        c.name.set("octoquad");
        c.xPodPort.set(0);
        c.yPodPort.set(1);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(1.0039370078740157);
        c.yPodOffset.set(-0.7874015748031497);
        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(12.274758379994843);
    });
    

//    public static PinpointConfig localizerConfig = new PinpointConfig(c -> { // TODO: REPLACE WITH REAL CONFIG
//        c.name.set("pinpoint");
//        c.xPodOffset.set(1.0039370078740157);
//        c.yPodOffset.set(-0.7874015748031497);
//        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSE);
//        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//    });

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.253439772563751);
                Controller secondaryTranslationalForward = Controller.proportional(0.09363922392686072);
                Controller primaryTranslationalLateral = Controller.proportional(0.26622416788094716);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09836271639123038);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01084687199104261));
                c.brake.set(Controller.proportionalFeedforward(0.009219841192386218));

                c.headingFeedback.set(Controller.proportional(0.3670227043337095));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.18208025384475668, -0.00166186882060501));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09931256088311197, 0.05847795565876414));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.944536269090321E-4, 0.0014574264679359317));

                c.maxAchievableForwardVelocity.set(77.04936467957937);
                c.maxAchievableStrafeVelocity.set(79.99687171681614);
                c.naturalForwardDeceleration.set(27.387533692656266);
                c.naturalStrafeDeceleration.set(51.05842233419405);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(new OctoQuadLocalizer(h,localizerConfig), new Mecanum(h,drivetrainConfig),new Foresight(foresightConfig));
//        return null;
    }
}