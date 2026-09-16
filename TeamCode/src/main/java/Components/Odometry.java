package Components;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Odometry {
    private final GoBildaPinpointDriver pinpoint;

    // Wiring: forward-tracking pod on the Pinpoint's X port, strafe-tracking pod on the Y
    // port - the driver's documented default. Re-verify with OdometryTest after wiring
    // changes: forward push should increase getForward(), right push should increase getRight().

    // Offset of the forward-tracking pod (X pod) from center, left (+) / right (-), in inches.
    // Measured via PodOffsetCalibration.
    private static final double FORWARD_POD_OFFSET = -0.752;
    // Offset of the strafe-tracking pod (Y pod) from center, forward (+) / backward (-), in inches.
    // Measured via PodOffsetCalibration.
    private static final double STRAFE_POD_OFFSET = -1.769;

    public Odometry(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(FORWARD_POD_OFFSET, STRAFE_POD_OFFSET, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
    }

    // Must be called once per loop to pull fresh data from the sensor.
    public void update() {
        pinpoint.update();
    }

    // Re-applies pod offsets without redeploying code, for calibration.
    public void setPodOffsets(double forwardPodOffset, double strafePodOffset, DistanceUnit unit) {
        pinpoint.setOffsets(forwardPodOffset, strafePodOffset, unit);
    }

    // Forward (+) / backward (-) distance traveled.
    public double getForward(DistanceUnit unit) {
        return pinpoint.getPosX(unit);
    }

    // Right (+) / left (-) distance traveled.
    public double getRight(DistanceUnit unit) {
        return pinpoint.getPosY(unit);
    }

    public double getHeading(AngleUnit unit) {
        return pinpoint.getHeading(unit);
    }

    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return pinpoint.getDeviceStatus();
    }
}
