package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import org.junit.After;
import org.junit.Test;
import static org.junit.Assert.*;

public class PollenCycleTimingTest {
    @After public void clear() { Scheduler.reset(); }

    @Test public void interruptsAnyActiveCycleAt25AndStopsParkingAt30() {
        double[] seconds = {0};
        boolean[] cleaned = {false}, parking = {false}, finished = {false};
        Command activeCycle = Command.build().setDone(() -> false)
                .setEnd(reason -> cleaned[0] = true);
        Command park = Command.build().setStart(() -> {
            assertTrue("Mechanisms must stop before parking", cleaned[0]);
            parking[0] = true;
        }).setDone(() -> false);
        Command routine = PedroPollenShootAndPark.routineCommand(() -> seconds[0],
                activeCycle, park, () -> finished[0] = true);
        Scheduler.schedule(routine);
        tick();
        seconds[0] = 24.99;
        tick();
        assertFalse(parking[0]);
        seconds[0] = 25;
        tick();
        assertTrue(parking[0]);
        assertFalse(finished[0]);
        seconds[0] = 29.99;
        tick();
        assertFalse(finished[0]);
        seconds[0] = 30;
        tick();
        assertTrue(finished[0]);
        assertFalse(Scheduler.isScheduled(routine));
    }

    @Test public void stopCancelsActiveCycleWithoutStartingParking() {
        boolean[] cleaned = {false}, parking = {false};
        Command routine = PedroPollenShootAndPark.routineCommand(() -> 10,
                Command.build().setDone(() -> false).setEnd(reason -> cleaned[0] = true),
                Command.build().setStart(() -> parking[0] = true), () -> {});
        Scheduler.schedule(routine);
        tick();
        Scheduler.cancel(routine);
        tick();
        assertTrue(cleaned[0]);
        assertFalse(parking[0]);
    }

    private void tick() {
        for (int i = 0; i < 8; i++) Scheduler.execute();
    }
}
