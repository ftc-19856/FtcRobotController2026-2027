package org.firstinspires.ftc.teamcode.opmode;

import org.junit.Test;
import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.opmode.FixedTurretIntakeShooterTeleOp.RelativeBlockerToggle;

public final class FixedTurretIntakeShooterTeleOpTest {
    @Test public void togglesByPointTwoAndReturnsToTheStartingCommand() {
        RelativeBlockerToggle toggle = new RelativeBlockerToggle(0.2, false, false);
        assertEquals(0.4, toggle.update(true, false), 1e-9);
        assertTrue(Double.isNaN(toggle.update(true, false))); // holding A cannot chatter
        toggle.update(false, false);
        assertEquals(0.2, toggle.update(true, false), 1e-9);
    }

    @Test public void outOfRangeMoveIsRejectedUntilTheOperatorReversesDirection() {
        RelativeBlockerToggle toggle = new RelativeBlockerToggle(0.9, false, false);
        assertTrue(Double.isNaN(toggle.update(true, false)));
        assertFalse(toggle.offsetActive);
        toggle.update(false, true);
        toggle.update(false, false);
        assertEquals(0.7, toggle.update(true, false), 1e-9);
        toggle.update(false, true); // B while offset must not switch the offset to the other side
        assertEquals(-1, toggle.direction);
        toggle.update(false, false);
        assertEquals(0.9, toggle.update(true, false), 1e-9);
    }

    @Test public void buttonsHeldAcrossStartDoNotMoveTheServo() {
        RelativeBlockerToggle toggle = new RelativeBlockerToggle(0.3, true, true);
        assertTrue(Double.isNaN(toggle.update(true, true)));
        assertTrue(Double.isNaN(toggle.update(true, false)));
        toggle.update(false, false);
        assertEquals(0.5, toggle.update(true, false), 1e-9);
    }

    @Test public void unknownOrInvalidStartingPositionDoesNotGuessAMove() {
        for (double start : new double[] {Double.NaN, Double.POSITIVE_INFINITY, -0.1, 1.1}) {
            RelativeBlockerToggle toggle = new RelativeBlockerToggle(start, false, false);
            assertTrue(Double.isNaN(toggle.update(true, false)));
            assertFalse(toggle.offsetActive);
        }
    }
}
