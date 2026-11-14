package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedro.AllianceAutonomousPaths;

/** Alliance autonomous that shoots preloads and then parks. */
public abstract class ShootLeaveParkAuto extends PedroPollenShootAndPark {
    private AllianceAutonomousPaths route;

    protected abstract AllianceAutonomousPaths createPaths();

    protected abstract double hiveY();

    @Override
    protected final double tunedShooterSpeedTicksPerSecond() {
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
