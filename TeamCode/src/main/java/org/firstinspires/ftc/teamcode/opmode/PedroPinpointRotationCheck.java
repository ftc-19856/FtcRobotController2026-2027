package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Checks the autonomous's actual Pinpoint configuration without mapping any motors. */
@TeleOp(name = "Pedro Pinpoint Rotation Check", group = "Test")
public final class PedroPinpointRotationCheck extends OpMode {
    private PinpointLocalizer localizer;
    private Pose startPose;
    private double previousHeading;
    private double totalTurn;
    private double maxDrift;

    @Override
    public void init() {
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        showTelemetry();
    }

    @Override
    public void init_loop() {
        localizer.update();
        showTelemetry();
    }

    @Override
    public void start() {
        localizer.update();
        Pose current = localizer.pose();
        startPose = new Pose(current.x(), current.y(), current.heading());
        previousHeading = current.heading();
        totalTurn = 0;
        maxDrift = 0;
    }

    @Override
    public void loop() {
        localizer.update();
        Pose current = localizer.pose();
        double change = current.heading() - previousHeading;
        totalTurn += Math.atan2(Math.sin(change), Math.cos(change));
        previousHeading = current.heading();
        maxDrift = Math.max(maxDrift,
                Math.hypot(current.x() - startPose.x(), current.y() - startPose.y()));
        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addData("Check revision", "rotation-offsets-r2");
        telemetry.addLine("No motors or servos are commanded by this check.");
        telemetry.addData("Pod type", Constants.localizerConfig.podType.get());
        telemetry.addData("X / Y pod offsets", "%.4f / %.4f",
                Constants.localizerConfig.xPodOffset.get(), Constants.localizerConfig.yPodOffset.get());
        telemetry.addData("Offset units", Constants.localizerConfig.offsetUnits.get());
        telemetry.addData("X / Y encoder direction", "%s / %s",
                Constants.localizerConfig.xPodDirection.get(), Constants.localizerConfig.yPodDirection.get());
        if (startPose == null) {
            telemetry.addLine("Keep still during INIT. Mark the robot center on the floor.");
        } else {
            Pose current = localizer.pose();
            telemetry.addLine("Rotate by HAND about the marked center, with BOTH pods on the floor.");
            telemetry.addLine("Pause at 90 and 180 degrees; X/Y change should remain near zero.");
            telemetry.addData("Turn since START (deg)", "%.2f", Math.toDegrees(totalTurn));
            telemetry.addData("X change (in)", "%.3f", current.x() - startPose.x());
            telemetry.addData("Y change (in)", "%.3f", current.y() - startPose.y());
            telemetry.addData("Maximum position drift (in)", "%.3f", maxDrift);
            telemetry.addData("Starting heading (deg)", "%.2f", Math.toDegrees(startPose.heading()));
            if (Math.abs(totalTurn) >= Math.toRadians(170)
                    && Math.abs(totalTurn) <= Math.toRadians(190)) {
                DistanceUnit units = Constants.localizerConfig.offsetUnits.get();
                double[] offsets = correctedOffsets(startPose, current,
                        units.toInches(Constants.localizerConfig.xPodOffset.get()),
                        units.toInches(Constants.localizerConfig.yPodOffset.get()));
                if (offsets != null) {
                    telemetry.addData("Candidate X pod offset (in)", "%.4f", offsets[0]);
                    telemetry.addData("Candidate Y pod offset (in)", "%.4f", offsets[1]);
                    telemetry.addLine("Candidates assume the physical center stayed on its mark.");
                    telemetry.addLine("Record these values. Configuration is NOT changed.");
                }
            } else {
                telemetry.addLine("Pause near 180 degrees to display candidate corrected offsets.");
            }
        }
        telemetry.update();
    }

    /** Corrects rotation-induced drift in the initial robot frame; all lengths are inches. */
    static double[] correctedOffsets(Pose start, Pose end, double configuredX, double configuredY) {
        double dx = end.x() - start.x(), dy = end.y() - start.y();
        double c = Math.cos(start.heading()), s = Math.sin(start.heading());
        double robotX = c * dx + s * dy;
        double robotY = -s * dx + c * dy;
        double turn = end.heading() - start.heading();
        double a = Math.sin(turn), b = 1 - Math.cos(turn);
        double denominator = a * a + b * b;
        if (!Double.isFinite(robotX) || !Double.isFinite(robotY)
                || !Double.isFinite(denominator) || denominator < 1) return null;
        // At exactly 180 degrees these reduce to subtracting robotY/2 from
        // the forward pod's lateral offset and robotX/2 from the strafe pod's offset.
        double xError = (a * robotX + b * robotY) / denominator;
        double yError = (b * robotX - a * robotY) / denominator;
        return new double[] {configuredX - xError, configuredY - yError};
    }
}
