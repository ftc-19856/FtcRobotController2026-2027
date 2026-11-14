package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Red start from the Visualizer with mirrored Blue shooting and parking geometry. */
public final class RedAlliancePaths extends AllianceAutonomousPaths {
    private static final PoseFactory POSES = PoseFactory.degrees();
    private static final Pose PARK_CONTROL_2 = POSES.of(14.0673, 69.0546, 0);

    public RedAlliancePaths() {
        super(POSES.of(8.8061, 32.1239, 90),
                POSES.of(53, 20, 90),
                POSES.of(0.0375, 46.4038, 0),
                POSES.of(11.3177, 96.0856, 95.8081));
    }

    @Override
    public Path path2() { return pathToPark(shooting); }

    @Override
    public Path pathToPark(Pose current) {
        return curve(current, parkControl, PARK_CONTROL_2, park).linear(current, park);
    }
}
