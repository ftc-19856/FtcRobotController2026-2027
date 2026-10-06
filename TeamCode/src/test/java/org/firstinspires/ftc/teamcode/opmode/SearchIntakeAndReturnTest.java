package org.firstinspires.ftc.teamcode.opmode;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

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
    public void headingHoldTurnsClockwiseWhenHeadingIsCounterClockwiseOfTarget() {
        assertEquals(0.15, SearchIntakeAndReturn.holdHeadingTurn(0.1, 0, 1.5, 0.25), EPSILON);
        assertEquals(-0.15, SearchIntakeAndReturn.holdHeadingTurn(-0.1, 0, 1.5, 0.25), EPSILON);
        assertEquals(0.25, SearchIntakeAndReturn.holdHeadingTurn(1, 0, 1.5, 0.25), EPSILON);
    }
}
