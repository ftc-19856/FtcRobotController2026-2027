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
        assertPose(route.start, 9.7, 32.8, 90);
        assertPose(route.path1().get(0), 9.7, 32.8, 90);
        assertPose(route.path1().get(1), 57.8, 27.3, 90);
        assertPose(route.path2().get(0), 57.8, 27.3, 90);
        assertPose(route.path2().get(0.5), 24.4877, 54.5706, 90);
        assertPose(route.path2().get(1), 9.7, 115, 90);
        assertConstantHeading(route.path2(), 90);
    }

    @Test public void bluePhysicallyStartsAt270AndKeepsThatHeadingOnBothPaths() {
        AllianceAutonomousPaths route = new BlueAlliancePaths();
        assertPose(route.start, 132, 109, 270);
        assertPose(route.path1().get(0), 132, 109, 270);
        assertPose(route.path1().get(1), 83.9, 108.6, 270);
        assertPose(route.path2().get(0), 83.9, 108.6, 270);
        assertPose(route.path2().get(0.5), 120.8048, 78.920775, 270);
        assertPose(route.path2().get(1), 135.409, 23.3501, 270);
        assertConstantHeading(route.path1(), 270);
        assertConstantHeading(route.path2(), 270);
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
