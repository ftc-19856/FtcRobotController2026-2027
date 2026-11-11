package org.firstinspires.ftc.teamcode.opmode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TuningFlywheelControlTest {
    @Test
    public void eitherEncoderSignProducesTheSameCorrection() {
        TuningFlywheelControl positive = new TuningFlywheelControl(2300);
        TuningFlywheelControl negative = new TuningFlywheelControl(2300);
        assertEquals(positive.power(1495, 1700, 0.02),
                negative.power(1495, -1700, 0.02), 0);
    }

    @Test
    public void raisingThenLoweringTheTargetChangesSpeedWithPositiveFeedbackReadings() {
        TuningFlywheelControl control = new TuningFlywheelControl(2300);
        double slower = settle(control, 1495, 0);
        double faster = settle(control, 1702, slower);
        double reduced = settle(control, 1495, faster);
        assertEquals(1495, slower, 20);
        assertEquals(1702, faster, 20);
        assertEquals(1495, reduced, 20);
        assertTrue(faster - slower > 150);
    }

    private double settle(TuningFlywheelControl control, double target, double speed) {
        for (int i = 0; i < 200; i++) {
            double power = control.power(target, speed, 0.02);
            assertTrue(power >= 0 && power <= 1);
            speed += (2200 * power - speed) * 0.08;
        }
        return speed;
    }
}
