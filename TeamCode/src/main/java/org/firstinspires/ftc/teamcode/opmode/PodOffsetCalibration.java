package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.localization.PinpointOdometry;

/** Estimates Pinpoint pod offsets while the robot is rotated by hand. */
@TeleOp(name = "Pod Offset Calibration", group = "Test")
public final class PodOffsetCalibration extends OpMode {
    private PinpointOdometry odometry;
    private double previousHeading;
    private double accumulatedRadians;
    private double startForward;
    private double startRight;
    private double sumRotationSquared;
    private double sumRotationForward;
    private double sumRotationRight;

    @Override
    public void init() {
        // Zero offsets are required so rotational encoder travel remains visible.
        odometry = new PinpointOdometry(hardwareMap, 0, 0);
        telemetry.addLine("Keep the robot still during initialization");
        telemetry.update();
    }

    @Override
    public void start() {
        odometry.update();
        previousHeading = odometry.getHeadingRadians();
        accumulatedRadians = 0;
        startForward = odometry.getForwardInches();
        startRight = odometry.getRightInches();
        sumRotationSquared = 0;
        sumRotationForward = 0;
        sumRotationRight = 0;
    }

    @Override
    public void loop() {
        odometry.update();
        double heading = odometry.getHeadingRadians();
        accumulatedRadians += wrapRadians(heading - previousHeading);
        previousHeading = heading;

        double forwardDrift = odometry.getForwardInches() - startForward;
        double rightDrift = odometry.getRightInches() - startRight;
        sumRotationSquared += accumulatedRadians * accumulatedRadians;
        sumRotationForward += accumulatedRadians * forwardDrift;
        sumRotationRight += accumulatedRadians * rightDrift;

        telemetry.addLine("Rotate by hand with both pods touching the floor");
        telemetry.addData("Accumulated turns", "%.2f",
                accumulatedRadians / (2 * Math.PI));
        if (sumRotationSquared > 0.25) {
            telemetry.addData("Candidate X offset (in)", "%.3f",
                    sumRotationForward / sumRotationSquared);
            telemetry.addData("Candidate Y offset (in)", "%.3f",
                    sumRotationRight / sumRotationSquared);
            telemetry.addLine("Copy these values into RobotConfig and verify with Odometry Test");
            telemetry.addLine("If rotational drift doubles, reverse that offset's sign");
        } else {
            telemetry.addLine("Rotate farther to calculate stable candidates");
        }
        telemetry.update();
    }

    static double wrapRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}
