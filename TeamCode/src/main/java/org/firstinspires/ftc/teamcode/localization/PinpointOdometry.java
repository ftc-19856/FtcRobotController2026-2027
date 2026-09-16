package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotConfig;

/** Configures and exposes the robot's two-wheel goBILDA Pinpoint odometry. */
public final class PinpointOdometry {
    private final GoBildaPinpointDriver pinpoint;

    public PinpointOdometry(HardwareMap hardwareMap) {
        this(hardwareMap, RobotConfig.PINPOINT_X_OFFSET_INCHES,
                RobotConfig.PINPOINT_Y_OFFSET_INCHES);
    }

    public PinpointOdometry(HardwareMap hardwareMap, double xOffset, double yOffset) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConfig.PINPOINT);
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public double getForwardInches() {
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    /** Returns the starter robot's verified lateral convention: right is positive. */
    public double getRightInches() {
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    public double getHeadingRadians() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return pinpoint.getDeviceStatus();
    }

    public void setOffsetsInches(double xOffset, double yOffset) {
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);
    }
}
