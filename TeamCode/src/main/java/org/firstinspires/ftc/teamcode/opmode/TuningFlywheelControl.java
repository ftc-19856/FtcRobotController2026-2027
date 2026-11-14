package org.firstinspires.ftc.teamcode.opmode;

/** Flywheel speed estimates and feedback from the measured encoder speed. */
final class TuningFlywheelControl {
    static final double FULL_SPEED_TICKS_PER_SECOND = 2300;
    private static final double PROPORTIONAL_GAIN = 0.7;
    private static final double INTEGRAL_GAIN = 1.2;

    private final double fullSpeed;
    private double previousTarget;
    private double integral;

    TuningFlywheelControl(double fullSpeed) {
        this.fullSpeed = fullSpeed;
    }

    static double speedForDistance(double distance) {
        if (!Double.isFinite(distance)) return 1495;
        double inches = Math.max(35, Math.min(50, distance));
        // Shallower-shooter trials: 61% at 35 in, 63% at 41 in, 68% at 50 in.
        if (inches <= 41) return 1403 + (1449 - 1403) * (inches - 35) / 6;
        return 1449 + (1564 - 1449) * (inches - 41) / 9;
    }

    double power(double target, double actual, double seconds) {
        if (target <= 0 || !Double.isFinite(actual)) {
            integral = 0;
            previousTarget = 0;
            return 0;
        }
        // Retain accumulated correction as a moving robot makes small target changes.
        if (Math.abs(target - previousTarget) > fullSpeed * 0.01) {
            integral = 0;
        }
        previousTarget = target;
        double error = (target - Math.abs(actual)) / fullSpeed;
        double base = target / fullSpeed + PROPORTIONAL_GAIN * error;
        double nextIntegral = integral + INTEGRAL_GAIN * error * Math.max(0, Math.min(0.1, seconds));
        double candidate = base + nextIntegral;
        // Stop accumulating error when the motor cannot apply any more correction.
        if ((candidate <= 1 || error < 0) && (candidate >= 0 || error > 0)) {
            integral = nextIntegral;
        }
        return Math.max(0, Math.min(1, base + integral));
    }
}
