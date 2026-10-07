package org.firstinspires.ftc.teamcode.opmode;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public final class SearchIntakeAndReturnTest {
    private static final double EPSILON = 1e-9;
    // Camera 7.75in up, ball center 1.425in up: 6.325in below the lens.
    private static final double DROP_INCHES = 6.325;

    private static double tyForDistanceAhead(double inches) {
        return -Math.toDegrees(Math.atan(DROP_INCHES / inches));
    }

    @Test
    public void ballStraightAheadIsCameraOffsetPlusDistance() {
        double[] ball = SearchIntakeAndReturn.ballFieldPosition(
                0, tyForDistanceAhead(30), 0, 0, 0);
        assertArrayEquals(new double[] {11.5 + 30, 0.4}, ball, EPSILON);
    }

    @Test
    public void positiveTxPutsTheBallToTheRight() {
        double[] ball = SearchIntakeAndReturn.ballFieldPosition(
                10, tyForDistanceAhead(30), 0, 0, 0);
        assertArrayEquals(
                new double[] {11.5 + 30, 0.4 + 30 * Math.tan(Math.toRadians(10))},
                ball, EPSILON);
    }

    @Test
    public void robotPoseIsAddedToTheBallPosition() {
        double[] ball = SearchIntakeAndReturn.ballFieldPosition(
                0, tyForDistanceAhead(30), 10, 5, 0);
        assertArrayEquals(new double[] {10 + 41.5, 5 + 0.4}, ball, EPSILON);
    }

    @Test
    public void ballAheadOfARobotFacingLeftIsToTheFieldLeft() {
        // Heading is counter-clockwise positive, so 90 degrees faces the field's left.
        double[] ball = SearchIntakeAndReturn.ballFieldPosition(
                0, tyForDistanceAhead(30), 0, 0, Math.PI / 2);
        assertArrayEquals(new double[] {0.4, -41.5}, ball, EPSILON);
    }

    @Test
    public void rejectsReadingsThatCannotBeABallOnTheFloorInTheField() {
        assertNull(SearchIntakeAndReturn.ballFieldPosition(0, 0, 0, 0, 0));
        assertNull(SearchIntakeAndReturn.ballFieldPosition(0, 5, 0, 0, 0));
        // 1 degree below the horizon is about 362in away, past the 144in field.
        assertNull(SearchIntakeAndReturn.ballFieldPosition(0, -1, 0, 0, 0));
    }

    @Test
    public void startingPoseTouchesButDoesNotLeaveTheSquare() {
        assertEquals(0, SearchIntakeAndReturn.footprintOverflow(0, 0, 0), EPSILON);
        // Backing up 5in pushes the back edge 5in past the back of the square.
        assertEquals(5, SearchIntakeAndReturn.footprintOverflow(-5, 0, 0), EPSILON);
    }

    @Test
    public void turningInPlaceInTheCornerWouldLeaveTheSquare() {
        // Facing left (90 degrees), the 11.5in front sticks 2.75in past the 8.75in left edge.
        assertEquals(2.75, SearchIntakeAndReturn.footprintOverflow(0, 0, Math.PI / 2), EPSILON);
    }

    @Test
    public void turnSafePointLeavesRoomToTurnAllTheWayAround() {
        for (int degrees = 0; degrees < 360; degrees += 15) {
            assertEquals(0, SearchIntakeAndReturn.footprintOverflow(
                    SearchIntakeAndReturn.TURN_SAFE_FORWARD, SearchIntakeAndReturn.TURN_SAFE_RIGHT,
                    Math.toRadians(degrees)), EPSILON);
        }
    }

    @Test
    public void targetsAreClampedInsideTheSquareForTheirHeading() {
        // Facing forward: front reaches 11.5in, square ends at 86, 2in margin -> 72.5.
        assertArrayEquals(new double[] {72.5, 40},
                SearchIntakeAndReturn.clampIntoSquare(100, 40, 0), EPSILON);
        // Facing right: front points at the right edge (87.25) -> 87.25 - 2 - 11.5.
        assertArrayEquals(new double[] {40, 73.75},
                SearchIntakeAndReturn.clampIntoSquare(40, 100, -Math.PI / 2), EPSILON);
        // Already inside: unchanged.
        assertArrayEquals(new double[] {40, 40},
                SearchIntakeAndReturn.clampIntoSquare(40, 40, 0), EPSILON);
    }

    @Test
    public void ballsOutsideTheSquareAreRejected() {
        assertTrue(SearchIntakeAndReturn.ballInsideSquare(50, 50));
        assertFalse(SearchIntakeAndReturn.ballInsideSquare(90, 10));
        assertFalse(SearchIntakeAndReturn.ballInsideSquare(10, -10));
    }

    @Test
    public void expectedBallSizeAtThirtyInchesAhead() {
        double range = Math.hypot(30, DROP_INCHES);
        assertEquals(0.853, SearchIntakeAndReturn.expectedBallAreaPercent(range), 0.005);
    }

    @Test
    public void detectionsMuchSmallerThanABallAreRejected() {
        double ty = tyForDistanceAhead(30);
        double expected = SearchIntakeAndReturn.expectedBallAreaPercent(Math.hypot(30, DROP_INCHES));
        // Real ball detections have measured 30-50% of the predicted size.
        assertTrue(SearchIntakeAndReturn.ballSizePlausible(0, ty, 0.4 * expected));
        assertFalse(SearchIntakeAndReturn.ballSizePlausible(0, ty, 0.1 * expected));
        assertFalse(SearchIntakeAndReturn.ballSizePlausible(0, 3, 1.0));
    }

    @Test
    public void headingHoldTurnsClockwiseWhenHeadingIsCounterClockwiseOfTarget() {
        assertEquals(0.15, SearchIntakeAndReturn.holdHeadingTurn(0.1, 0, 1.5, 0.25), EPSILON);
        assertEquals(-0.15, SearchIntakeAndReturn.holdHeadingTurn(-0.1, 0, 1.5, 0.25), EPSILON);
        assertEquals(0.25, SearchIntakeAndReturn.holdHeadingTurn(1, 0, 1.5, 0.25), EPSILON);
    }
}
