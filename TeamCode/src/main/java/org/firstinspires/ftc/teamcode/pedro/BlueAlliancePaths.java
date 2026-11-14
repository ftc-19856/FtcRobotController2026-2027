package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Blue alliance routes; the physical starting heading is confirmed as 270 degrees. */
public final class BlueAlliancePaths extends AllianceAutonomousPaths {
    private static final PoseFactory POSES = PoseFactory.degrees();

    public BlueAlliancePaths() {
        super(POSES.of(132, 109, 270),
                POSES.of(87, 108, 270),
                POSES.of(139, 97, 0),
                POSES.of(135.409, 23.3501, 270));
    }

    @Override
    public Path pathToPark(Pose current) {
        return BlueHiveParking.route(current, park, parkControl);
    }
}
