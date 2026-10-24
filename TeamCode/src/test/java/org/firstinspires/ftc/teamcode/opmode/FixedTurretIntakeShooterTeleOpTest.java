package org.firstinspires.ftc.teamcode.opmode;

import org.junit.Test;
import com.pedropathing.drivetrain.DrivePowers;
import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.opmode.FixedTurretIntakeShooterTeleOp.RelativeBlockerToggle;

public final class FixedTurretIntakeShooterTeleOpTest {
    @Test public void drivingStickDirectionsMatchPedroRobotCentricAxes() {
        DrivePowers forward = FixedTurretIntakeShooterTeleOp.drivingPowers(0, -1, 0, false);
        assertEquals(1, forward.forward(), 1e-9);
        assertEquals(0, forward.strafe(), 1e-9);
        assertEquals(0, forward.turn(), 1e-9);
        DrivePowers right = FixedTurretIntakeShooterTeleOp.drivingPowers(1, 0, 0, false);
        assertEquals(-1, right.strafe(), 1e-9); // Pedro strafe is left-positive
        DrivePowers clockwise = FixedTurretIntakeShooterTeleOp.drivingPowers(0, 0, 1, false);
        assertEquals(-1, clockwise.turn(), 1e-9); // Pedro turn is counter-clockwise-positive
    }

    @Test public void slowModeCapsCombinedFullStickInputsAfterNormalization() {
        DrivePowers full = FixedTurretIntakeShooterTeleOp.drivingPowers(1, -1, 1, false);
        DrivePowers slow = FixedTurretIntakeShooterTeleOp.drivingPowers(1, -1, 1, true);
        assertEquals(1, Math.abs(full.forward()) + Math.abs(full.strafe()) + Math.abs(full.turn()), 1e-9);
        assertEquals(0.35, Math.abs(slow.forward()) + Math.abs(slow.strafe()) + Math.abs(slow.turn()), 1e-9);
        assertEquals(full.forward() * 0.35, slow.forward(), 1e-9);
        assertEquals(full.strafe() * 0.35, slow.strafe(), 1e-9);
        assertEquals(full.turn() * 0.35, slow.turn(), 1e-9);
    }

    @Test public void centeredStickNoiseAndNonFiniteInputsDoNotDrive() {
        for (double input : new double[] {0, 0.02, -0.02, Double.NaN, Double.POSITIVE_INFINITY}) {
            DrivePowers powers = FixedTurretIntakeShooterTeleOp.drivingPowers(input, input, input, false);
            assertEquals(0, powers.forward(), 1e-9);
            assertEquals(0, powers.strafe(), 1e-9);
            assertEquals(0, powers.turn(), 1e-9);
        }
    }

    @Test public void togglesByPointTwoAndReturnsToTheStartingCommand() {
        RelativeBlockerToggle toggle = new RelativeBlockerToggle(0.2, false, false);
        assertEquals(0.0, toggle.update(true, false), 1e-9);
        assertTrue(Double.isNaN(toggle.update(true, false))); // holding A cannot chatter
        toggle.update(false, false);
        assertEquals(0.2, toggle.update(true, false), 1e-9);
    }

    @Test public void outOfRangeMoveIsRejectedUntilTheOperatorReversesDirection() {
        RelativeBlockerToggle toggle = new RelativeBlockerToggle(0.1, false, false);
        assertTrue(Double.isNaN(toggle.update(true, false)));
        assertFalse(toggle.offsetActive);
        toggle.update(false, true);
        toggle.update(false, false);
        assertEquals(0.3, toggle.update(true, false), 1e-9);
        toggle.update(false, true); // B while offset must not switch the offset to the other side
        assertEquals(1, toggle.direction);
        toggle.update(false, false);
        assertEquals(0.1, toggle.update(true, false), 1e-9);
    }

    @Test public void buttonsHeldAcrossStartDoNotMoveTheServo() {
        RelativeBlockerToggle toggle = new RelativeBlockerToggle(0.3, true, true);
        assertTrue(Double.isNaN(toggle.update(true, true)));
        assertTrue(Double.isNaN(toggle.update(true, false)));
        toggle.update(false, false);
        assertEquals(0.1, toggle.update(true, false), 1e-9);
    }

    @Test public void unknownOrInvalidStartingPositionDoesNotGuessAMove() {
        for (double start : new double[] {Double.NaN, Double.POSITIVE_INFINITY, -0.1, 1.1}) {
            RelativeBlockerToggle toggle = new RelativeBlockerToggle(start, false, false);
            assertTrue(Double.isNaN(toggle.update(true, false)));
            assertFalse(toggle.offsetActive);
        }
    }
}
