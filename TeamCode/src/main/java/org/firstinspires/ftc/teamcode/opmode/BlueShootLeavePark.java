package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedro.AllianceAutonomousPaths;
import org.firstinspires.ftc.teamcode.pedro.BlueAlliancePaths;

@Autonomous(name = "Blue Shoot Leave Park", group = "Competition")
public final class BlueShootLeavePark extends ShootLeaveParkAuto {
    @Override
    protected AllianceAutonomousPaths createPaths() {
        return new BlueAlliancePaths();
    }
}
