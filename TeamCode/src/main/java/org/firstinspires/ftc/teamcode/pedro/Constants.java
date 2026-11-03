package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left");
        c.frontRightName.set("front_right");
        c.backLeftName.set("back_left");
        c.backRightName.set("back_right");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(6.6900028018500866);
        c.yPodOffset.set(4.972861883208507);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.27717670347708845);
                Controller secondaryTranslationalForward = Controller.proportional(0.10240938563686354);
                Controller primaryTranslationalLateral = Controller.proportional(0.2602531503660166);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09615658496799);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.019596591598751354));
                c.brake.set(Controller.proportionalFeedforward(0.01665710285893865));

                c.headingFeedback.set(Controller.proportional(3.1725332306456613));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.044475268982814634, 0.004340144746620712));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05521381509184382, 0.0381469806826949));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012934449194278114, 0.0017209331744808224));

                c.maxAchievableForwardVelocity.set(60.55370451492705);
                c.maxAchievableStrafeVelocity.set(45.704395459134815);
                c.naturalForwardDeceleration.set(55.88453469262021);
                c.naturalStrafeDeceleration.set(67.49947688958254);
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