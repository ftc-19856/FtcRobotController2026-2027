package org.firstinspires.ftc.teamcode.opmode;

import org.junit.Test;
import static org.junit.Assert.*;

public class PollenPassCounterTest {
    @Test public void countsOnlyAfterYellowClearsAndDoesNotRepeatWhileClear() {
        PollenPassCounter counter = new PollenPassCounter();
        counter.update(false, true, 0);
        counter.update(false, true, 0.1);
        assertEquals(0, counter.count());
        pass(counter, 1);
        counter.update(false, true, 5);
        assertEquals(1, counter.count());
        pass(counter, 6);
        assertEquals(2, counter.count());
    }

    @Test public void rejectsFlickerAndIgnoresFeeding() {
        PollenPassCounter counter = new PollenPassCounter();
        counter.update(false, true, 0);
        counter.update(false, true, 0.1);
        counter.update(true, true, 0.2);
        counter.update(false, true, 0.21);
        assertEquals(0, counter.count());
        counter.update(false, false, 1);
        counter.update(true, false, 2);
        counter.update(false, true, 3);
        counter.update(false, true, 4);
        assertEquals(0, counter.count());
        pass(counter, 5);
        assertEquals(1, counter.count());
    }

    @Test public void capacityIsFourAndNextBatchStartsEmpty() {
        PollenPassCounter counter = new PollenPassCounter();
        for (int i = 0; i < 6; i++) pass(counter, i);
        assertEquals(4, counter.count());
        counter.reset();
        assertEquals(0, counter.count());
        pass(counter, 10);
        assertEquals(1, counter.count());
    }

    @Test public void yellowRequiresBothRedAndGreenAboveBlue() {
        assertTrue(PedroPollenShootAndPark.isYellow(200, 180, 40));
        assertFalse(PedroPollenShootAndPark.isYellow(200, 30, 40));
        assertFalse(PedroPollenShootAndPark.isYellow(200, 200, 200));
        assertFalse(PedroPollenShootAndPark.isYellow(0, 0, 0));
    }

    private void pass(PollenPassCounter counter, double time) {
        counter.update(true, true, time);
        counter.update(true, true, time + 0.05);
        counter.update(false, true, time + 0.1);
        counter.update(false, true, time + 0.15);
    }
}
