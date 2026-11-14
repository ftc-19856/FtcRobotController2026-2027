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
    protected boolean holdTurretDuringAuto() { return false; }

    @Override
    protected double hiveY() { return 83; }

    @Override
    protected double tunedShooterSpeedTicksPerSecond() {
        return 0.59 * TuningFlywheelControl.FULL_SPEED_TICKS_PER_SECOND;
    }

    @Override
    protected AllianceAutonomousPaths createPaths() {
        return new BlueAlliancePaths();
    }
}
