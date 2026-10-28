package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedro.AllianceAutonomousPaths;
import org.firstinspires.ftc.teamcode.pedro.BlueAlliancePaths;

@Autonomous(name = "Blue Shoot Leave Park", group = "Competition")
public final class BlueShootLeavePark extends ShootLeaveParkAuto {
    static final double MINIMUM_FIELD_X = 70;

    @Override
    protected double minimumFieldX() { return MINIMUM_FIELD_X; }

    @Override
    protected AllianceAutonomousPaths createPaths() {
        return new BlueAlliancePaths();
    }
}
