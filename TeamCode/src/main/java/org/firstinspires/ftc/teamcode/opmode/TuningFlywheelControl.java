package org.firstinspires.ftc.teamcode.opmode;

/** Velocity feedback for the tuning OpMode using the magnitude of the encoder reading. */
final class TuningFlywheelControl {
    private static final double PROPORTIONAL_GAIN = 0.7;
    private static final double INTEGRAL_GAIN = 1.2;

    private final double fullSpeed;
    private double previousTarget;
    private double integral;

    TuningFlywheelControl(double fullSpeed) {
        this.fullSpeed = fullSpeed;
    }

    double power(double target, double actual, double seconds) {
        if (target <= 0 || !Double.isFinite(actual)) {
            integral = 0;
            previousTarget = 0;
            return 0;
        }
        if (target != previousTarget) {
            integral = 0;
            previousTarget = target;
        }
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
