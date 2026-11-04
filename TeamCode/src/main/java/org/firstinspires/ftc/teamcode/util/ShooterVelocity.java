package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Shared Hub velocity PIDF for the competition shooter. */
@Configurable
public final class ShooterVelocity {
    // Set these in Panels after the first INIT, which loads the Hub's current gains.
    public static double P;
    public static double I;
    public static double D;
    public static double F;
    public static double SPEED_FRACTION = 1.0;
    private static boolean defaultsLoaded;

    private final DcMotorEx motor;
    private final double maximumTicksPerSecond;
    private boolean enabled;
    private double target;
    private double appliedP = Double.NaN;
    private double appliedI = Double.NaN;
    private double appliedD = Double.NaN;
    private double appliedF = Double.NaN;
    private boolean settingsValid = true;

    public ShooterVelocity(DcMotorEx motor) {
        this.motor = motor;
        motor.setPower(0);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        maximumTicksPerSecond = motor.getMotorType().getTicksPerRev()
                * motor.getMotorType().getMaxRPM() / 60.0;
        if (!Double.isFinite(maximumTicksPerSecond) || maximumTicksPerSecond <= 0) {
            throw new IllegalArgumentException("Configure a valid motor type for shooter");
        }
        if (!defaultsLoaded) {
            PIDFCoefficients existing = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            P = existing.p;
            I = existing.i;
            D = existing.d;
            F = existing.f;
            defaultsLoaded = true;
        }
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        update();
    }

    /** The Hub runs the PID loop; this refreshes commands and live Panels settings. */
    public void update() {
        double p = P, i = I, d = D, f = F, fraction = SPEED_FRACTION;
        settingsValid = Double.isFinite(p) && p >= 0 && Double.isFinite(i) && i >= 0
                && Double.isFinite(d) && d >= 0 && Double.isFinite(f) && f >= 0
                && Double.isFinite(fraction) && fraction >= 0 && fraction <= 1;
        if (!enabled || !settingsValid || fraction == 0) {
            target = 0;
            // Power zero preserves FLOAT spin-down rather than actively regulating zero velocity.
            motor.setPower(0);
            return;
        }
        if (p != appliedP || i != appliedI || d != appliedD || f != appliedF) {
            motor.setVelocityPIDFCoefficients(p, i, d, f);
            appliedP = p;
            appliedI = i;
            appliedD = d;
            appliedF = f;
        }
        // Negative matches the previously working shooter.setPower(-1).
        target = -Math.min(maximumTicksPerSecond * fraction, 32767);
        motor.setVelocity(target);
    }

    public void stop() {
        enabled = false;
        target = 0;
        motor.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        double actual = motor.getVelocity();
        telemetry.addData("Shooter target / actual (ticks/s)", "%.0f / %.0f", target, actual);
        telemetry.addData("Shooter error (ticks/s)", "%.0f", target - actual);
        telemetry.addData("Shooter PIDF", "%.3f / %.3f / %.3f / %.3f", P, I, D, F);
        telemetry.addData("Shooter PIDF settings", settingsValid ? "VALID" : "INVALID: shooter stopped");
    }
}
