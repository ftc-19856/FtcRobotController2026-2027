package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedro.AllianceAutonomousPaths;
import org.firstinspires.ftc.teamcode.pedro.RedAlliancePaths;

@Autonomous(name = "Red Shoot Leave Park", group = "Competition")
public final class RedShootLeavePark extends ShootLeaveParkAuto {
    @Override
    protected double hiveY() { return 57; }

    @Override
    protected double tunedShooterSpeedTicksPerSecond() {
        return 0.59 * TuningFlywheelControl.FULL_SPEED_TICKS_PER_SECOND;
    }

    @Override
    protected AllianceAutonomousPaths createPaths() {
        return new RedAlliancePaths();
    }
}
