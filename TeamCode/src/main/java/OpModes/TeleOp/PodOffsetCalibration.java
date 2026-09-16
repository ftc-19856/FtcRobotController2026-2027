package OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Components.Odometry;

// Calibrates dead wheel pod offsets by spinning the robot in place and measuring how
// far each pod's reading drifts relative to the actual rotation (from the Pinpoint's
// own IMU heading). A pod offset from the true center of rotation shows up as spurious
// forward/strafe "movement" proportional to (offset * rotation angle), so offset =
// drift / rotation gives the offset directly from encoder ticks - far more precise
// than measuring with a ruler.
//
// This does NOT drive the wheels - motor-driven in-place rotation tends to walk/drift
// across the floor (uneven wheel friction), which contaminates the measurement. Instead,
// spin the robot BY HAND while it stays on the floor (dead wheel pods must stay in
// contact with the ground to register anything - lifting the robot into the air gives
// no data at all). Keep the chassis's center roughly fixed over one spot as you spin it,
// like spinning it on a lazy susan; the mecanum wheels will just skid unpowered, which is fine.
//
// Usage: Init, then Start, then spin the robot by hand for several rotations. Watch the
// "candidate" offsets in telemetry; they'll converge as more rotation accumulates. Press
// A to apply the candidate offsets live and start phase 2, which keeps tracking while you
// spin more and shows the resulting drift - it should stay near zero. If it doesn't,
// press X to flip the sign and compare again. Once the post-apply drift is flat, copy the
// shown offsets into Components/Odometry.java.
@TeleOp(name = "PodOffsetCalibration", group = "Test")
public class PodOffsetCalibration extends OpMode {
    Odometry odometry;

    double prevHeading;
    double accumulatedRadians;

    double startForward;
    double startRight;

    // Running least-squares regression (through the origin) of drift vs. rotation,
    // accumulated over every sample collected during the spin. This averages out
    // random hand-spin wobble far better than comparing just the start/end snapshot.
    double sumRotationSquared;
    double sumRotationTimesForwardDrift;
    double sumRotationTimesRightDrift;

    boolean applied;
    boolean flipped;
    double appliedForwardOffset;
    double appliedStrafeOffset;
    double postApplyBaselineForward;
    double postApplyBaselineRight;

    @Override
    public void init() {
        // Offsets start at whatever's currently in Odometry.java (should still be 0,0
        // until this calibration is done).
        odometry = new Odometry(hardwareMap);

        telemetry.addData("Status", "Initialized. Hold the robot still, then Start.");
        telemetry.update();
    }

    @Override
    public void start() {
        odometry.update();
        prevHeading = odometry.getHeading(AngleUnit.RADIANS);
        accumulatedRadians = 0;
        startForward = odometry.getForward(DistanceUnit.INCH);
        startRight = odometry.getRight(DistanceUnit.INCH);
        sumRotationSquared = 0;
        sumRotationTimesForwardDrift = 0;
        sumRotationTimesRightDrift = 0;
        applied = false;
        flipped = false;
    }

    @Override
    public void loop() {
        odometry.update();

        double heading = odometry.getHeading(AngleUnit.RADIANS);
        double delta = heading - prevHeading;
        if (delta > Math.PI) delta -= 2 * Math.PI;
        if (delta < -Math.PI) delta += 2 * Math.PI;
        accumulatedRadians += delta;
        prevHeading = heading;

        double forward = odometry.getForward(DistanceUnit.INCH);
        double right = odometry.getRight(DistanceUnit.INCH);

        if (!applied) {
            double driftForward = forward - startForward;
            double driftRight = right - startRight;

            // Fold this sample into the running regression.
            sumRotationSquared += accumulatedRadians * accumulatedRadians;
            sumRotationTimesForwardDrift += accumulatedRadians * driftForward;
            sumRotationTimesRightDrift += accumulatedRadians * driftRight;

            double turns = accumulatedRadians / (2 * Math.PI);
            telemetry.addLine("Spin the robot BY HAND, on the floor, pods touching the ground.");
            telemetry.addData("Turns so far", "%.2f", turns);

            if (sumRotationSquared > 0.25) {
                double candidateForwardOffset = sumRotationTimesForwardDrift / sumRotationSquared;
                double candidateStrafeOffset = sumRotationTimesRightDrift / sumRotationSquared;

                telemetry.addLine("Keep spinning back and forth - more samples improve the fit.");
                telemetry.addData("Candidate FORWARD_POD_OFFSET (in)", "%.3f", candidateForwardOffset);
                telemetry.addData("Candidate STRAFE_POD_OFFSET (in)", "%.3f", candidateStrafeOffset);
                telemetry.addLine("Press A to apply these live and verify.");
            } else {
                telemetry.addLine("Keep spinning a bit more before candidates show up.");
            }

            if (gamepad1.a && sumRotationSquared > 0.25) {
                appliedForwardOffset = sumRotationTimesForwardDrift / sumRotationSquared;
                appliedStrafeOffset = sumRotationTimesRightDrift / sumRotationSquared;
                odometry.setPodOffsets(appliedForwardOffset, appliedStrafeOffset, DistanceUnit.INCH);
                postApplyBaselineForward = odometry.getForward(DistanceUnit.INCH);
                postApplyBaselineRight = odometry.getRight(DistanceUnit.INCH);
                applied = true;
            }
        } else {
            double postDriftForward = forward - postApplyBaselineForward;
            double postDriftRight = right - postApplyBaselineRight;

            telemetry.addData("Applied FORWARD_POD_OFFSET (in)", "%.3f", appliedForwardOffset);
            telemetry.addData("Applied STRAFE_POD_OFFSET (in)", "%.3f", appliedStrafeOffset);
            telemetry.addLine("Keep spinning by hand - drift below should stay near zero if correct.");
            telemetry.addData("Post-apply forward drift (in)", "%.3f", postDriftForward);
            telemetry.addData("Post-apply right drift (in)", "%.3f", postDriftRight);

            if (Math.abs(postDriftForward) + Math.abs(postDriftRight) > 1.0 && !flipped) {
                telemetry.addLine("Drift is growing - press X to try flipping the sign.");
            }

            if (gamepad1.x && !flipped) {
                appliedForwardOffset = -appliedForwardOffset;
                appliedStrafeOffset = -appliedStrafeOffset;
                odometry.setPodOffsets(appliedForwardOffset, appliedStrafeOffset, DistanceUnit.INCH);
                postApplyBaselineForward = odometry.getForward(DistanceUnit.INCH);
                postApplyBaselineRight = odometry.getRight(DistanceUnit.INCH);
                flipped = true;
            }

            telemetry.addLine("Once drift stays flat, copy these two values into Odometry.java.");
        }

        telemetry.update();
    }
}
