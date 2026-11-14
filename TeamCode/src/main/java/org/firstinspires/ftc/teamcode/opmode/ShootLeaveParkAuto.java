package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.pedro.AllianceAutonomousPaths;

/** Alliance autonomous that shoots preloads and then parks. */
public abstract class ShootLeaveParkAuto extends PedroPollenShootAndPark {
    private static final int MAX_TRIM_POINTS = 10;
    private static final double TICKS_PER_POINT = TuningFlywheelControl.FULL_SPEED_TICKS_PER_SECOND / 100;
    private AllianceAutonomousPaths route;
    private final Gamepad previousButtons = new Gamepad();
    private int trimPoints;

    @Override
    public final void init() {
        trimPoints = 0;
        previousButtons.copy(gamepad1);
        super.init();
    }

    @Override
    public final void init_loop() {
        if (gamepad1.x && !previousButtons.x) {
            trimPoints = 0;
        } else {
            int change = (gamepad1.dpad_up && !previousButtons.dpad_up ? 1 : 0)
                    - (gamepad1.dpad_down && !previousButtons.dpad_down ? 1 : 0)
                    + (gamepad1.dpad_right && !previousButtons.dpad_right ? 5 : 0)
                    - (gamepad1.dpad_left && !previousButtons.dpad_left ? 5 : 0);
            trimPoints = Math.max(-MAX_TRIM_POINTS, Math.min(MAX_TRIM_POINTS, trimPoints + change));
        }
        previousButtons.copy(gamepad1);
        double base = baseShooterTarget();
        double target = base + shooterTrimTicksPerSecond();
        telemetry.addData("Flywheel speed", "Base %.0f%% | Trim %+d points | Target %.0f%% (%.0f ticks/s)",
                base / TICKS_PER_POINT, trimPoints, target / TICKS_PER_POINT, target);
        telemetry.addLine("D-pad up/down adjusts 1 point. Right/left adjusts 5. X resets.");
        super.init_loop();
    }

    @Override
    protected final double shooterTrimTicksPerSecond() {
        return trimPoints * TICKS_PER_POINT;
    }

    protected abstract AllianceAutonomousPaths createPaths();

    protected abstract double hiveY();

    @Override
    protected double tunedShooterSpeedTicksPerSecond() {
        Pose shoot = shootingTargetPose();
        return TuningFlywheelControl.speedForDistance(Math.hypot(83 - shoot.x(), hiveY() - shoot.y()));
    }

    @Override
    protected final boolean seekPollen() { return false; }

    private AllianceAutonomousPaths route() {
        if (route == null) route = createPaths();
        return route;
    }

    @Override
    protected final Pose startingPose() { return route().start; }

    @Override
    protected final Pose shootingTargetPose() { return route().shooting; }

    @Override
    protected final Pose parkingPose() { return route().park; }

    @Override
    protected final Path parkingPath(Pose current) { return route().pathToPark(current); }
}
