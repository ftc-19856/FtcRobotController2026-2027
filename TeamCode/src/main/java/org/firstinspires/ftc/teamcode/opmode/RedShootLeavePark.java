package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedro.AllianceAutonomousPaths;
import org.firstinspires.ftc.teamcode.pedro.RedAlliancePaths;

@Autonomous(name = "Red Shoot Leave Park", group = "Competition")
public final class RedShootLeavePark extends ShootLeaveParkAuto {
    @Override
    protected AllianceAutonomousPaths createPaths() {
        return new RedAlliancePaths();
    }
}
