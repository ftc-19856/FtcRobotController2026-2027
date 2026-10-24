package org.firstinspires.ftc.teamcode.opmode;

import org.junit.Test;
import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.opmode.FixedTurretIntakeShooterTeleOp.BlockerToggle;

public final class FixedTurretIntakeShooterTeleOpTest {
    @Test public void unknownStartingPositionStillProducesAnExplicitServoCommand() {
        BlockerToggle toggle = new BlockerToggle(Double.NaN, false, false);
        assertEquals(0.2, toggle.update(true, false), 1e-9);
        assertTrue(Double.isNaN(toggle.update(true, false)));
        toggle.update(false, false);
        assertEquals(0, toggle.update(true, false), 1e-9);
    }

    @Test public void knownOpenPositionTogglesClosedAndBAlwaysCommandsBlocking() {
        BlockerToggle toggle = new BlockerToggle(0.2, false, false);
        assertEquals(0, toggle.update(true, false), 1e-9);
        toggle.update(false, false);
        assertEquals(0.2, toggle.update(true, false), 1e-9);
        assertEquals(0, toggle.update(true, true), 1e-9);
        assertTrue(Double.isNaN(toggle.update(true, true)));
    }

    @Test public void heldButtonsAcrossStartDoNotMoveUntilAFreshPress() {
        BlockerToggle toggle = new BlockerToggle(Double.NaN, true, true);
        assertTrue(Double.isNaN(toggle.update(true, true)));
        toggle.update(false, false);
        assertEquals(0, toggle.update(true, true), 1e-9); // B wins a simultaneous press
    }
}
