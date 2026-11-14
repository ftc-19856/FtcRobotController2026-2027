package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.RedAlliancePaths;

@Configurable
@TeleOp(name = "Red Pedro Turret TeleOp", group = "Competition")
public final class RedPedroTurretTeleOp extends AlliancePedroTurretTeleOp {
    public RedPedroTurretTeleOp() {
        super("Red", new RedAlliancePaths().start, 57, 57, 83);
    }
}
