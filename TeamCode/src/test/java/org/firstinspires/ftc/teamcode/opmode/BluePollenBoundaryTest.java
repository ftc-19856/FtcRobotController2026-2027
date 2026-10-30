package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedro.BlueAlliancePaths;
import org.junit.Test;
import static org.junit.Assert.*;

public class BluePollenBoundaryTest {
    private static final double BOUNDARY = BlueShootLeavePark.MINIMUM_FIELD_X;

    @Test public void searchAllowsRoomForTheWholeRobotToRotateAboveX70() {
        Pose anchor = PedroPollenShootAndPark.clearancePose(new Pose(75, 80, 0), BOUNDARY);
        assertTrue(anchor.x() - Math.hypot(9, 9) > 70);
        assertEquals(80, anchor.y(), 1e-9);
    }

    @Test public void ignoresPollenAcrossBoundaryAndUnreachablePickupTargets() {
        Pose robot = new Pose(100, 80, Math.PI);
        assertNull(PedroPollenShootAndPark.pickupPose(robot, new Pose(69, 80), BOUNDARY));
        assertNull(PedroPollenShootAndPark.pickupPose(robot, new Pose(72, 80), BOUNDARY));
        Pose goal = PedroPollenShootAndPark.pickupPose(robot, new Pose(90, 80), BOUNDARY);
        assertNotNull(goal);
        assertTrue(goal.x() - Math.hypot(9, 9) > 70);
    }

    @Test public void startAndParkAreWithinBlueBoundsAndRedRemainsUnrestricted() {
        BlueAlliancePaths blue = new BlueAlliancePaths();
        double centerLimit = BOUNDARY + PedroPollenShootAndPark.TURN_CLEARANCE;
        assertTrue(blue.start.x() >= centerLimit);
        assertTrue(blue.park.x() >= centerLimit);
        // Straight return/park paths remain within the half-plane when both endpoints do.
        Pose red = PedroPollenShootAndPark.clearancePose(new Pose(30, 80));
        assertEquals(30, red.x(), 1e-9);
    }

    @Test public void straightShootingApproachDoesNotUseDiagonalFootprint() {
        // Previously this normal 1.4-inch endpoint overshoot permanently aborted auto.
        assertFalse(PedroPollenShootAndPark.outsideSideBoundary(
                new Pose(82.5, 108.6, Math.toRadians(270)), BOUNDARY));
        assertTrue(PedroPollenShootAndPark.outsideSideBoundary(
                new Pose(79, 108.6, Math.toRadians(270)), BOUNDARY));
        assertTrue(PedroPollenShootAndPark.outsideSideBoundary(
                new Pose(82.5, 108.6, Math.toRadians(45)), BOUNDARY));
        assertFalse(PedroPollenShootAndPark.outsideSideBoundary(
                new Pose(20, 80, 0), 0));
    }
}
