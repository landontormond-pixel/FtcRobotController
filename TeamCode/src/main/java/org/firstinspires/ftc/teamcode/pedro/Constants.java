package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FLDrive");
        c.frontRightName.set("FRDrive");
        c.backLeftName.set("BLDrive");
        c.backRightName.set("BRDrive");
        c.frontLeftDirection.set(DcMotorEx.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorEx.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorEx.Direction.REVERSE);
        c.backRightDirection.set(DcMotorEx.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("Pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.318925872562439);
        c.yPodOffset.set(-2.975353931817483);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2684605573570851);
                Controller secondaryTranslationalForward = Controller.proportional(0.09918900254523601);
                Controller primaryTranslationalLateral = Controller.proportional(0.382874796573128);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14146200673078665);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.020245354604922102));
                c.brake.set(Controller.proportionalFeedforward(0.017208551414183787));

                c.headingFeedback.set(Controller.proportional(4.0612223432535215));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05022206933594271, 0.006219601586086261));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05439760698625031, 0.07184061726191128));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0021902881686584135, 0.00122377028519496));

                c.maxAchievableForwardVelocity.set(42.508227133064544);
                c.maxAchievableStrafeVelocity.set(35.735535561830744);
                c.naturalForwardDeceleration.set(39.125324345628776);
                c.naturalStrafeDeceleration.set(65.42342624172838);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}