package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.BlueAlliancePaths;

@Configurable
@TeleOp(name = "Blue Pedro Turret TeleOp", group = "Competition")
public final class BluePedroTurretTeleOp extends AlliancePedroTurretTeleOp {
    public BluePedroTurretTeleOp() {
        super("Blue", new BlueAlliancePaths().start, 83, 83, 55);
    }
}
