package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.junit.Test;
import static org.junit.Assert.*;

public class BlueHiveParkingTest {
    private final BlueAlliancePaths routes = new BlueAlliancePaths();

    @Test public void parksFromEitherSideAndAboveOrBelowHiveWithTurningClearance() {
        for (Pose start : new Pose[] {routes.shooting,
                new Pose(75, 75, 0), new Pose(75, 125, Math.PI),
                new Pose(75, 20, Math.PI / 2), new Pose(85, 20, 0),
                new Pose(120, 75, Math.PI), new Pose(105, 120, Math.PI / 4)}) {
            Path route = routes.pathToPark(start);
            assertNotNull(route);
            assertEquals(start.x(), route.get(0).x(), 1e-6);
            assertEquals(start.y(), route.get(0).y(), 1e-6);
            assertEquals(routes.park.x(), route.get(1).x(), 1e-6);
            assertEquals(routes.park.y(), route.get(1).y(), 1e-6);
            assertEquals(0, wrapped(route.get(1).heading() - routes.park.heading()), 1e-6);
            for (int i = 0; i <= 2000; i++) {
                Pose point = route.get(i / 2000.0);
                assertTrue("Route entered hive turning buffer: " + point, BlueHiveParking.outside(point));
                assertTrue("Blue center boundary crossed", point.x() > 70);
            }
        }
    }

    @Test public void movesAwayWithoutTurningBeforeLeavingConservativeBuffer() {
        Pose start = new Pose(83, 75, Math.toRadians(270));
        Path route = routes.pathToPark(start);
        assertNotNull(route);
        for (int i = 0; i <= 2000; i++) {
            Pose p = route.get(i / 2000.0);
            if (!BlueHiveParking.outside(p)) {
                assertEquals(start.y(), p.y(), 1e-6);
                assertTrue(p.x() <= start.x() + 1e-6);
                assertEquals(0, wrapped(p.heading() - start.heading()), 1e-6);
            }
        }
    }

    @Test public void refusesPathWhenRobotAlreadyOverlapsTheStructure() {
        assertNull(routes.pathToPark(new Pose(95, 70, 0)));
    }

    @Test public void lineIntersectionRecognizesBlockedCrossingsAndClearCornerEdges() {
        assertFalse(BlueHiveParking.clearSegment(new Pose(75, 75), new Pose(120, 75)));
        assertTrue(BlueHiveParking.clearSegment(new Pose(75, 120), new Pose(120, 120)));
        assertTrue(BlueHiveParking.clearSegment(new Pose(BlueHiveParking.LEFT, BlueHiveParking.TOP),
                new Pose(BlueHiveParking.RIGHT, BlueHiveParking.TOP)));
    }

    private static double wrapped(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
