package org.firstinspires.ftc.teamcode.drive;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive.WheelPowers;
import org.junit.Test;

public final class MecanumDriveTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void forwardDrivesAllWheelsForward() {
        WheelPowers powers = MecanumDrive.calculate(1, 0, 0);
        assertPowers(powers, 1, 1, 1, 1);
    }

    @Test
    public void rightStrafeUsesMecanumPattern() {
        WheelPowers powers = MecanumDrive.calculate(0, 1, 0);
        assertPowers(powers, 1, -1, -1, 1);
    }

    @Test
    public void clockwiseTurnUsesOppositeSides() {
        WheelPowers powers = MecanumDrive.calculate(0, 0, 1);
        assertPowers(powers, 1, 1, -1, -1);
    }

    @Test
    public void combinedInputsAreNormalized() {
        WheelPowers powers = MecanumDrive.calculate(1, 1, 1);
        assertPowers(powers, 1, 1.0 / 3.0, -1.0 / 3.0, 1.0 / 3.0);
    }

    @Test
    public void fieldForwardAtZeroYawDrivesRobotForward() {
        WheelPowers powers = MecanumDrive.calculateFieldCentric(1, 0, 0, 0);
        assertPowers(powers, 1, 1, 1, 1);
    }

    @Test
    public void fieldForwardAtPositiveQuarterTurnStrafesRobotRight() {
        WheelPowers powers = MecanumDrive.calculateFieldCentric(
                1, 0, 0, Math.PI / 2);
        assertPowers(powers, 1, -1, -1, 1);
    }

    @Test
    public void fieldForwardAtNegativeQuarterTurnStrafesRobotLeft() {
        WheelPowers powers = MecanumDrive.calculateFieldCentric(
                1, 0, 0, -Math.PI / 2);
        assertPowers(powers, -1, 1, 1, -1);
    }

    private static void assertPowers(
            WheelPowers powers, double frontLeft, double backLeft,
            double frontRight, double backRight) {
        assertEquals(frontLeft, powers.frontLeft, EPSILON);
        assertEquals(backLeft, powers.backLeft, EPSILON);
        assertEquals(frontRight, powers.frontRight, EPSILON);
        assertEquals(backRight, powers.backRight, EPSILON);
    }
}
