package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;

/** Blue alliance routes; the physical starting heading is confirmed as 270 degrees. */
public final class BlueAlliancePaths extends AllianceAutonomousPaths {
    private static final PoseFactory POSES = PoseFactory.degrees();

    public BlueAlliancePaths() {
        super(POSES.of(132, 109, 270),
                POSES.of(83.9, 108.6, 270),
                POSES.of(131.9551, 91.8665, 0),
                POSES.of(135.409, 23.3501, 270));
    }
}
