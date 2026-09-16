package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive.WheelPowers;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp(name = "Starter TeleOp", group = "Competition")
public final class StarterTeleOp extends OpMode {
    private MecanumDrive drive;
    private DcMotorEx intake;
    private final Toggle intakeToggle = new Toggle(false);

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        try {
            intake = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR);
            intake.setPower(0);
        } catch (IllegalArgumentException exception) {
            intake = null;
            telemetry.addLine("Warning: intake motor is not configured");
        }
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        drive.resetYaw();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x;
        WheelPowers powers = drive.driveFieldCentric(forward, right, clockwise);

        intakeToggle.update(gamepad1.a);
        if (intake != null) {
            intake.setPower(intakeToggle.get() ? 1.0 : 0.0);
        }

        telemetry.addData("Yaw (rad)", "%.3f", drive.getYawRadians());
        telemetry.addData("Drive FL/FR", "%.2f / %.2f",
                powers.frontLeft, powers.frontRight);
        telemetry.addData("Drive BL/BR", "%.2f / %.2f",
                powers.backLeft, powers.backRight);
        telemetry.addData("Intake", intake == null ? "not configured"
                : (intakeToggle.get() ? "on" : "off"));
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        if (intake != null) {
            intake.setPower(0);
        }
    }
}
