package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedro.AllianceAutonomousPaths;
import org.firstinspires.ftc.teamcode.pedro.RedAlliancePaths;

@Autonomous(name = "Red Path Wait Park", group = "Competition")
public final class RedPathAuto extends AlliancePathAuto {
    @Override
    protected AllianceAutonomousPaths createPaths() {
        return new RedAlliancePaths();
    }
}
