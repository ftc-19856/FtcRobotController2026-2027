package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2613707766960264);
                Controller secondaryTranslationalForward = Controller.proportional(0.09656951803340309);
                Controller primaryTranslationalLateral = Controller.proportional(0.3081397301762981);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1138493966549106);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017108575630937935));
                c.brake.set(Controller.proportionalFeedforward(0.014542289286297245));

                c.headingFeedback.set(Controller.proportional(6.226418351112599));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.053937335033154406, 0.005560843511452094));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06106355691311051, 0.07987991978407859));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0018389665317833223, 9.214721568133191E-4));

                c.maxAchievableForwardVelocity.set(57.87666546978052);
                c.maxAchievableStrafeVelocity.set(50.08221173419087);
                c.naturalForwardDeceleration.set(34.67362684010035);
                c.naturalStrafeDeceleration.set(57.70148014866875);
            }
    );

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftMotor");
        c.frontRightName.set("frontRightMotor");
        c.backLeftName.set("backLeftMotor");
        c.backRightName.set("backRightMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.650030181163879);
        c.yPodOffset.set(-9.229864886426551);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
}
