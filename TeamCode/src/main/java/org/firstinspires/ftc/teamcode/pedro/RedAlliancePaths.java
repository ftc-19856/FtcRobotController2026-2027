package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;

/** Red alliance routes exported from the Pedro Visualizer. */
public final class RedAlliancePaths extends AllianceAutonomousPaths {
    private static final PoseFactory POSES = PoseFactory.degrees();

    public RedAlliancePaths() {
        super(POSES.of(9.7, 32.8, 90),
                POSES.of(57.8, 27.3, 90),
                POSES.of(15.2254, 37.9912, 0),
                POSES.of(9.7, 115, 90));
    }
}
