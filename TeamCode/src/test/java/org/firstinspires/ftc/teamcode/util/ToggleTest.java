package org.firstinspires.ftc.teamcode.util;

import org.junit.Test;
import static org.junit.Assert.*;

public final class ToggleTest {
    @Test public void heldButtonAcrossStartRequiresReleaseAndAnotherPress() {
        Toggle toggle = new Toggle(false, true);
        toggle.update(true);
        assertFalse(toggle.get());
        toggle.update(false);
        toggle.update(true);
        assertTrue(toggle.get());
        toggle.update(true);
        assertTrue(toggle.get());
        toggle.update(false);
        toggle.update(true);
        assertFalse(toggle.get());
    }

    @Test public void intakeAndShooterToggleIndependently() {
        Toggle intake = new Toggle(false);
        Toggle shooter = new Toggle(false);
        intake.update(true);
        shooter.update(false);
        assertTrue(intake.get());
        assertFalse(shooter.get());
        shooter.update(true);
        assertTrue(intake.get());
        assertTrue(shooter.get());
        intake.update(false);
        intake.update(true);
        assertFalse(intake.get());
        assertTrue(shooter.get());
    }
}
