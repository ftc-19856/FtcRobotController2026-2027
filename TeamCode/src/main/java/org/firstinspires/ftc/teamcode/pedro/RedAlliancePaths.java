package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;

/** Red alliance routes exported from the Pedro Visualizer. */
public final class RedAlliancePaths extends AllianceAutonomousPaths {
    private static final PoseFactory POSES = PoseFactory.degrees();

    public RedAlliancePaths() {
        super(POSES.of(8.8061, 32.1239, 90),
                POSES.of(53.7261, 32.1083, 90),
                POSES.of(6.455, 49.9691, 0),
                POSES.of(9.5244, 117.7155, 90));
    }
}
