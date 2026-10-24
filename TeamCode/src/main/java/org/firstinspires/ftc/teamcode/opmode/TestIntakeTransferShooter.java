package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestITS", group = "Tests")
public class TestIntakeTransferShooter extends OpMode {
    DcMotor i;
    DcMotor s;

    @Override
    public void init() {
        i = hardwareMap.get(DcMotor.class, "intake");
        s = hardwareMap.get(DcMotor.class, "shooter");

    }

    @Override
    public void loop() {
        i.setPower(-1);
        s.setPower(-1);
    }
}
