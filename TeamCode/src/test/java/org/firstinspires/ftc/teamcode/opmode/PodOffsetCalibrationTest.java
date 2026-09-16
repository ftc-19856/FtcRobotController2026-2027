package org.firstinspires.ftc.teamcode.opmode;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public final class PodOffsetCalibrationTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void wrapsAcrossPositivePi() {
        assertEquals(Math.toRadians(2),
                PodOffsetCalibration.wrapRadians(Math.toRadians(-358)), EPSILON);
    }

    @Test
    public void wrapsAcrossNegativePi() {
        assertEquals(Math.toRadians(-2),
                PodOffsetCalibration.wrapRadians(Math.toRadians(358)), EPSILON);
    }

    @Test
    public void autonomousHeadingCorrectionOpposesHeadingError() {
        assertEquals(-0.15, IntakeAndReturn.headingCorrection(0.1), EPSILON);
        assertEquals(0.15, IntakeAndReturn.headingCorrection(-0.1), EPSILON);
        assertEquals(-0.25, IntakeAndReturn.headingCorrection(1), EPSILON);
    }
}
