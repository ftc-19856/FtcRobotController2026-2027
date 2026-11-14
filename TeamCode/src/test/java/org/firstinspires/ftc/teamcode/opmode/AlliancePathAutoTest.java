package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedro.AllianceAutonomousPaths;
import org.firstinspires.ftc.teamcode.pedro.RedAlliancePaths;
import org.firstinspires.ftc.teamcode.pedro.BlueAlliancePaths;
import org.junit.After;
import org.junit.Test;
import static org.junit.Assert.*;

public final class AlliancePathAutoTest {
    @After public void clearScheduler() { Scheduler.reset(); }

    @Test public void redUsesTheSuppliedLineAndCurveWithPedroDegreeHeadings() {
        AllianceAutonomousPaths route = new RedAlliancePaths();
        assertPose(route.start, 8.8061, 32.1239, 90);
        assertPose(route.path1().get(0), 8.8061, 32.1239, 90);
        assertPose(route.path1().get(1), 53, 20, 90);
        assertPose(route.path2().get(0), 53, 20, 90);
        assertEquals(13.3290125, route.path2().get(0.5).x(), 1e-6);
        assertEquals(57.8076, route.path2().get(0.5).y(), 1e-6);
        assertPose(route.path2().get(1), 11.3177, 96.0856, 95.8081);
        assertPose(new RedShootLeavePark().shootingTargetPose(), 53, 20, 90);
        assertConstantHeading(route.path1(), 90);
        for (int i = 0; i <= 1000; i++) {
            Pose sample = route.path2().get(i / 1000.0);
            double halfWidth = 9 * (Math.abs(Math.cos(sample.heading())) + Math.abs(Math.sin(sample.heading())));
            assertTrue("Red parking curve clears the left wall", sample.x() - halfWidth > 1.4);
        }
    }

    @Test public void blueUsesTheCubicParkingCurveAndExportedHeading() {
        AllianceAutonomousPaths route = new BlueAlliancePaths();
        assertPose(route.start, 132, 109, 270);
        assertPose(route.path1().get(0), 132, 109, 270);
        assertPose(route.path1().get(1), 87, 120, 270);
        assertPose(route.path2().get(0), 87, 120, 270);
        assertEquals(126.6709875, route.path2().get(0.5).x(), 1e-6);
        assertEquals(82.1924, route.path2().get(0.5).y(), 1e-6);
        assertPose(route.path2().get(1), 128.6823, 43.9144, -84.1919);
        assertConstantHeading(route.path1(), 270);
        Path dynamic = route.pathToPark(route.shooting);
        assertPose(dynamic.get(0.5), 126.6709875, 82.1924,
                Math.toDegrees(route.path2().get(0.5).heading()));
    }

    @Test public void parkingFromAnOffsetPoseRetainsBothBlueControlPoints() {
        AllianceAutonomousPaths route = new BlueAlliancePaths();
        Path path = route.pathToPark(new Pose(120, 100, Math.toRadians(180)));
        assertPose(path.get(0), 120, 100, 180);
        // Cubic midpoint weights the four points 1:3:3:1.
        Pose middle = path.get(0.5);
        assertEquals(130.7959875, middle.x(), 1e-6);
        assertEquals(79.6924, middle.y(), 1e-6);
        // Pedro interpolates heading over distance, not the Bezier parameter.
        assertTrue(middle.heading() > Math.toRadians(180));
        assertTrue(middle.heading() < Math.toRadians(275.8081));
        assertPose(path.get(1), 128.6823, 43.9144, -84.1919);
    }

    @Test public void redParkingFromShootingPoseMatchesOriginalSuppliedCurve() {
        for (AllianceAutonomousPaths route : new AllianceAutonomousPaths[] {
                new RedAlliancePaths()}) {
            Path dynamic = route.pathToPark(route.shooting);
            for (double t : new double[] {0, 0.25, 0.5, 0.75, 1}) {
                Pose original = route.path2().get(t);
                assertPose(dynamic.get(t), original.x(), original.y(),
                        Math.toDegrees(original.heading()));
            }
        }
    }

    @Test public void ivyWaitsForEachPhaseBeforeStartingTheNextOne() {
        boolean[] completed = {false, false, false};
        java.util.List<String> events = new java.util.ArrayList<>();
        Command routine = AlliancePathAuto.routineCommand(
                phase(events, "drive", completed, 0),
                phase(events, "wait", completed, 1),
                phase(events, "park", completed, 2), () -> events.add("done"));
        Scheduler.schedule(routine);
        tick();
        assertEquals(java.util.Arrays.asList("drive"), events);
        completed[0] = true;
        tick();
        assertEquals(java.util.Arrays.asList("drive", "wait"), events);
        completed[1] = true;
        tick();
        assertEquals(java.util.Arrays.asList("drive", "wait", "park"), events);
        completed[2] = true;
        tick();
        assertEquals(java.util.Arrays.asList("drive", "wait", "park", "done"), events);
        assertFalse(Scheduler.isScheduled(routine));
    }

    @Test public void cancellingDuringTheWaitDoesNotStartParking() {
        java.util.List<String> events = new java.util.ArrayList<>();
        boolean[] completed = {true, false, false};
        Command routine = AlliancePathAuto.routineCommand(
                phase(events, "drive", completed, 0),
                phase(events, "wait", completed, 1),
                phase(events, "park", completed, 2), () -> events.add("done"));
        Scheduler.schedule(routine);
        tick();
        Scheduler.cancel(routine);
        completed[1] = true;
        tick();
        assertEquals(java.util.Arrays.asList("drive", "wait"), events);
        assertFalse(Scheduler.isScheduled(routine));
    }

    private static Command phase(java.util.List<String> events, String name,
            boolean[] done, int index) {
        return Command.build().setStart(() -> events.add(name)).setDone(() -> done[index]);
    }

    private static void tick() {
        for (int i = 0; i < 5; i++) Scheduler.execute();
    }

    private static void assertConstantHeading(Path path, double degrees) {
        for (double t : new double[] {0, 0.25, 0.5, 0.75, 1}) {
            double error = path.get(t).heading() - Math.toRadians(degrees);
            assertEquals(0, Math.atan2(Math.sin(error), Math.cos(error)), 1e-9);
        }
    }

    private static void assertPose(Pose pose, double x, double y, double degrees) {
        assertEquals(x, pose.x(), 1e-6);
        assertEquals(y, pose.y(), 1e-6);
        double error = pose.heading() - Math.toRadians(degrees);
        assertEquals(0, Math.atan2(Math.sin(error), Math.cos(error)), 1e-9);
    }
}
