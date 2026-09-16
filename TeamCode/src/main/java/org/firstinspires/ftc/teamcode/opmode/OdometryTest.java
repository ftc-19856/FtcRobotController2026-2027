package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.localization.PinpointOdometry;

/** Drives the robot while showing live Pinpoint pose and direction checks. */
@TeleOp(name = "Odometry Test", group = "Test")
public final class OdometryTest extends OpMode {
    private MecanumDrive drive;
    private PinpointOdometry odometry;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        odometry = new PinpointOdometry(hardwareMap);
        telemetry.addLine("Initialized; keep still while the Pinpoint IMU calibrates");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.driveRobotCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x);
        odometry.update();

        telemetry.addData("Pinpoint", odometry.getStatus());
        telemetry.addData("Forward X (in)", "%.3f", odometry.getForwardInches());
        telemetry.addData("Right Y (in)", "%.3f", odometry.getRightInches());
        telemetry.addData("Heading (deg)", "%.2f",
                Math.toDegrees(odometry.getHeadingRadians()));
        telemetry.addLine("Forward motion must increase X");
        telemetry.addLine("Right motion must increase Y");
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
