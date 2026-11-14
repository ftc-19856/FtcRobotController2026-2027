package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Blue alliance routes; the physical starting heading is confirmed as 270 degrees. */
public final class BlueAlliancePaths extends AllianceAutonomousPaths {
    private static final PoseFactory POSES = PoseFactory.degrees();
    private static final Pose PARK_CONTROL_2 = POSES.of(125.9327, 70.9454, 0);

    public BlueAlliancePaths() {
        super(POSES.of(132, 109, 270),
                POSES.of(87, 120, 270),
                POSES.of(139.9625, 93.5962, 0),
                POSES.of(128.6823, 43.9144, -84.1919));
    }

    @Override
    public Path path2() {
        return curve(shooting, parkControl, PARK_CONTROL_2, park).linear(shooting, park);
    }

    @Override
    public Path pathToPark(Pose current) {
        return BlueHiveParking.route(current, park, parkControl, PARK_CONTROL_2);
    }
}
