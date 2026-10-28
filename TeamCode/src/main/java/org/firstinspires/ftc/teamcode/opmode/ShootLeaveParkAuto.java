package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedro.AllianceAutonomousPaths;

/** Alliance starts/parks with the shared preload, seek-four, return-and-shoot cycle. */
public abstract class ShootLeaveParkAuto extends PedroPollenShootAndPark {
    private AllianceAutonomousPaths route;

    protected abstract AllianceAutonomousPaths createPaths();

    private AllianceAutonomousPaths route() {
        if (route == null) route = createPaths();
        return route;
    }

    @Override
    protected final Pose startingPose() { return route().start; }

    @Override
    protected final Pose parkingPose() { return route().park; }
}
