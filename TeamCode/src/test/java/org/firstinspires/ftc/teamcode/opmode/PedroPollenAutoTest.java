package org.firstinspires.ftc.teamcode.opmode;

import org.junit.Test;
import org.junit.After;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static org.junit.Assert.*;

public class PedroPollenAutoTest {
    @After public void clearScheduler() { Scheduler.reset(); }

    @Test public void turnOnlyHoldCommandsNoTranslationEvenWhenOdometryDrifts() {
        ForesightConfig config = Constants.foresightConfig;
        double previousTranslation = config.holdPointTranslationalScaling.get();
        double previousHeading = config.holdPointHeadingScaling.get();
        try {
            PedroPollenAuto.configureTurnOnlyHolds(config);
            MotionState reported = MotionState.ofTwist(new Pose(55, 52, 0), new Twist(2, -1, 0.15));
            Pose target = new Pose(50, 50, 0.5);
            Drivetrain drivetrain = new TestDrivetrain();
            DrivePowers turnOnly = new Foresight(config).calculateHold(
                    drivetrain, target, reported, true, 0.02);
            assertEquals(0, turnOnly.forward(), 1e-9);
            assertEquals(0, turnOnly.strafe(), 1e-9);
            assertTrue(turnOnly.turn() > 0);

            // Unscaled holds (including the park hold) still correct position.
            DrivePowers normalHold = new Foresight(config).calculateHold(
                    drivetrain, target, reported, false, 0.02);
            assertTrue(Math.hypot(normalHold.forward(), normalHold.strafe()) > 0.01);
        } finally {
            config.holdPointTranslationalScaling.set(previousTranslation);
            config.holdPointHeadingScaling.set(previousHeading);
        }
    }

    private static final class TestDrivetrain implements Drivetrain {
        public void drive(DrivePowers powers, boolean manual) {}
        public void stop() {}
        public void stop(boolean brake) {}
        public double maxScaling(DrivePowers current, DrivePowers delta) { return 1; }
        public double interpolateVelocity(double forward, double lateral, double angle) { return 50; }
        public java.util.Map<String, Object> debug() { return java.util.Collections.emptyMap(); }
    }

    @Test public void startsAtCapacityAndCannotCollectWithFourPollen() {
        assertEquals(4, PedroPollenAuto.STARTING_POLLEN);
        assertEquals(4, PedroPollenAuto.POLLEN_CAPACITY);
        assertFalse(PedroPollenAuto.hasCapacity(PedroPollenAuto.STARTING_POLLEN));
        assertFalse(PedroPollenAuto.hasCapacity(5));
        assertFalse(PedroPollenAuto.hasCapacity(-1));
        assertTrue(PedroPollenAuto.hasCapacity(0));
        assertTrue(PedroPollenAuto.hasCapacity(3));
    }

    @Test public void pickupEstimatesStopAtFourWithoutFreeingPreloadCapacity() {
        int estimate = 0;
        for (int expected = 1; expected <= 4; expected++) {
            estimate = PedroPollenAuto.estimateAfterPickup(estimate);
            assertEquals(expected, estimate);
        }
        assertEquals(4, PedroPollenAuto.estimateAfterPickup(estimate));
        assertFalse(PedroPollenAuto.hasCapacity(estimate));
        assertEquals(4, PedroPollenAuto.estimateAfterPickup(PedroPollenAuto.STARTING_POLLEN));
    }

    @Test public void ivyStopsTrackingAndCleansUpAtTwentySeconds() {
        double[] time = {0};
        int[] calls = {0, 0, 0}; // start, track, cleanup
        Command collect = PedroPollenAuto.collectCommand(() -> time[0],
                () -> calls[0]++, () -> calls[1]++, () -> calls[2]++);
        Scheduler.schedule(collect);
        time[0] = 19.999;
        Scheduler.execute();
        assertEquals(1, calls[0]);
        assertTrue(calls[1] > 0);
        int tracksBeforeDeadline = calls[1];
        time[0] = 20;
        Scheduler.execute();
        assertEquals(tracksBeforeDeadline, calls[1]);
        assertEquals(1, calls[2]);
        assertFalse(Scheduler.isScheduled(collect));
    }

    @Test public void ivyCancellingCollectionRunsCleanup() {
        int[] cleanup = {0};
        Command collect = PedroPollenAuto.collectCommand(() -> 5,
                () -> {}, () -> {}, () -> cleanup[0]++);
        Scheduler.schedule(collect);
        Scheduler.execute();
        Scheduler.cancel(collect);
        assertEquals(1, cleanup[0]);
        assertFalse(Scheduler.isScheduled(collect));
    }

    @Test public void ivySequencesCollectionThenParkWithGlobalThirtySecondDeadline() {
        double[] time = {0};
        int[] calls = {0, 0, 0}; // tracking, parking, final cleanup
        Command collect = PedroPollenAuto.collectCommand(() -> time[0],
                () -> {}, () -> calls[0]++, () -> {});
        Command park = infinite(() -> calls[1]++);
        Command routine = PedroPollenAuto.routineCommand(() -> time[0], collect,
                park, () -> calls[2]++);
        Scheduler.schedule(routine);
        for (int i = 0; i < 5; i++) Scheduler.execute();
        assertTrue(calls[0] > 0);
        assertEquals(0, calls[1]);
        time[0] = 20;
        for (int i = 0; i < 5; i++) Scheduler.execute();
        assertTrue(calls[1] > 0);
        assertEquals(0, calls[2]);
        time[0] = 30;
        for (int i = 0; i < 5; i++) Scheduler.execute();
        assertEquals(1, calls[2]);
        assertFalse(Scheduler.isScheduled(routine));
    }

    @Test public void cameraProjectionUsesPedroFieldCoordinatesAndRightPositiveTx() {
        double ty = -Math.toDegrees(Math.atan((8.5 - 2.85 / 2) / 40));
        Pose ball = PedroPollenAuto.ballFieldPose(new Pose(50, 50, 0), 10, ty);
        assertNotNull(ball);
        double right = 40 * Math.tan(Math.toRadians(10));
        assertEquals(97.5, ball.x(), 1e-6);
        assertEquals(50 - right, ball.y(), 1e-6);
        Pose rotated = PedroPollenAuto.ballFieldPose(new Pose(50, 50, Math.PI / 2), 10, ty);
        assertEquals(50 + right, rotated.x(), 1e-6);
        assertEquals(97.5, rotated.y(), 1e-6);
        assertNull(PedroPollenAuto.ballFieldPose(new Pose(140, 140, 0), 0, ty));
    }

    @Test public void targetLockNeedsDistinctConsistentFrames() {
        PedroPollenAuto.TargetSamples samples = new PedroPollenAuto.TargetSamples();
        samples.add(new Pose(50, 50), 1);
        samples.add(new Pose(50, 50), 1);
        samples.add(new Pose(51, 50), 2);
        assertNull(samples.agreed());
        samples.add(new Pose(50, 51), 3);
        assertEquals(50, samples.agreed().x(), 1e-6);
        assertEquals(50, samples.agreed().y(), 1e-6);
        samples.reset();
        assertNull(samples.agreed());
        samples.add(new Pose(50, 50), 1);
        samples.add(new Pose(50, 50), 2);
        samples.add(new Pose(80, 80), 3);
        assertNull(samples.agreed());
    }

    @Test public void robotMovesInsideTheCornerBeforeSpinning() {
        Pose clear = PedroPollenAuto.clearancePose(new Pose(132.4, 9.8, Math.PI / 2));
        assertTrue(clear.x() < 132.4);
        assertTrue(clear.y() > 9.8);
        assertEquals(Math.PI / 2, clear.heading(), 1e-6);
        // Check the entire 18x18 footprint through a full rotation.
        for (int degrees = 0; degrees < 360; degrees += 5) {
            double c = Math.cos(Math.toRadians(degrees)), s = Math.sin(Math.toRadians(degrees));
            for (int front : new int[] {-9, 9}) {
                for (int left : new int[] {-9, 9}) {
                    double x = clear.x() + front * c - left * s;
                    double y = clear.y() + front * s + left * c;
                    assertTrue(x >= 1 - 1e-6 && x <= 143 + 1e-6);
                    assertTrue(y >= 1 - 1e-6 && y <= 143 + 1e-6);
                }
            }
        }
    }

    @Test public void pickupGoalFacesPollenAndTravelsPastIntakeContact() {
        Pose goal = PedroPollenAuto.pickupPose(new Pose(50, 40, 0), new Pose(50, 80));
        assertNotNull(goal);
        assertEquals(50, goal.x(), 1e-6);
        assertEquals(74, goal.y(), 1e-6);
        assertEquals(Math.PI / 2, goal.heading(), 1e-6);
        assertNull(PedroPollenAuto.pickupPose(new Pose(50, 50, 0), new Pose(143, 50)));
        assertNull(PedroPollenAuto.pickupPose(new Pose(50, 50, 0), new Pose(50, 50)));
    }

    @Test public void rejectsImpossibleCameraRanges() {
        assertTrue(Double.isNaN(PedroPollenAuto.cameraRange(0)));
        assertTrue(Double.isNaN(PedroPollenAuto.cameraRange(5)));
        assertTrue(Double.isNaN(PedroPollenAuto.cameraRange(-0.01)));
        assertTrue(Double.isNaN(PedroPollenAuto.cameraRange(Double.NaN)));
        assertTrue(Double.isNaN(PedroPollenAuto.cameraRange(-90)));
        double ty = -Math.toDegrees(Math.atan((8.5 - 2.85 / 2) / 40));
        assertEquals(40, PedroPollenAuto.cameraRange(ty), 1e-6);
    }

    @Test public void gardenIsAbout84InchesStraightAheadOfTheStartInsideTheField() {
        assertTrue(garden.gardenMeasured());
        // The start faces +y (90 degrees), so straight ahead is up the field.
        assertEquals(84, garden.GARDEN_Y - garden.START_Y, 0.1);
        assertEquals(0, garden.GARDEN_X - garden.START_X, 0.5);
        assertEquals(90, Math.toDegrees(garden.GARDEN_HEADING), 5);
        assertTrue(garden.GARDEN_X + 9 <= 144 && garden.GARDEN_Y + 9 <= 144);
    }
}
