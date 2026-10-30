package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.math.Pose;
import org.junit.Test;
import static org.junit.Assert.*;

public class PedroPinpointRotationCheckTest {
    @Test public void halfTurnCorrectsExistingOffsetsInsteadOfAssumingTheyWereZero() {
        double[] result = PedroPinpointRotationCheck.correctedOffsets(
                Pose.zero(), new Pose(-25, 5, Math.PI), -6.65, -9.23);
        assertArrayEquals(new double[] {-9.15, 3.27}, result, 1e-9);
    }

    @Test public void correctionIsIndependentOfInitialFieldHeadingAndPosition() {
        // Same initial-robot-frame drift (-25,5), with the robot initially facing +Y.
        double[] result = PedroPinpointRotationCheck.correctedOffsets(
                new Pose(83, 109, Math.PI / 2),
                new Pose(78, 84, 3 * Math.PI / 2), -6.65, -9.23);
        assertArrayEquals(new double[] {-9.15, 3.27}, result, 1e-9);
    }

    @Test public void usesMeasuredAngleForTurnsSlightlyPast180() {
        double angle = Math.toRadians(180.94);
        // Known offset errors (2.5,-12.5) integrated through this turn.
        double dx = 2.5 * Math.sin(angle) - 12.5 * (1 - Math.cos(angle));
        double dy = 2.5 * (1 - Math.cos(angle)) + 12.5 * Math.sin(angle);
        double[] result = PedroPinpointRotationCheck.correctedOffsets(
                Pose.zero(), new Pose(dx, dy, angle), -6.65, -9.23);
        assertArrayEquals(new double[] {-9.15, 3.27}, result, 1e-9);
    }

    @Test public void noDriftPreservesCalibrationAndTinyTurnsAreRejected() {
        assertArrayEquals(new double[] {-6.65, -9.23},
                PedroPinpointRotationCheck.correctedOffsets(
                        Pose.zero(), new Pose(0, 0, -Math.PI), -6.65, -9.23), 1e-9);
        assertNull(PedroPinpointRotationCheck.correctedOffsets(
                Pose.zero(), new Pose(0, 0, 0.01), -6.65, -9.23));
    }
}
