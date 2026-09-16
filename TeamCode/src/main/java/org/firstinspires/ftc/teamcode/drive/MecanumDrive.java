package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConfig;

/** Shared robot-centric and field-centric mecanum drivetrain. */
public final class MecanumDrive {
    private final DcMotorEx frontLeft;
    private final DcMotorEx backLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backRight;
    private final IMU imu;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, RobotConfig.FRONT_LEFT_MOTOR);
        backLeft = hardwareMap.get(DcMotorEx.class, RobotConfig.BACK_LEFT_MOTOR);
        frontRight = hardwareMap.get(DcMotorEx.class, RobotConfig.FRONT_RIGHT_MOTOR);
        backRight = hardwareMap.get(DcMotorEx.class, RobotConfig.BACK_RIGHT_MOTOR);
        imu = hardwareMap.get(IMU.class, RobotConfig.IMU);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientation));

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        configureMotor(frontLeft);
        configureMotor(backLeft);
        configureMotor(frontRight);
        configureMotor(backRight);
    }

    private static void configureMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    /** Drives in robot coordinates: forward, right strafe, clockwise turn. */
    public WheelPowers driveRobotCentric(double forward, double right, double clockwise) {
        WheelPowers powers = calculate(forward, right, clockwise);
        apply(powers);
        return powers;
    }

    /** Drives in starting-field coordinates while using the Control Hub IMU yaw. */
    public WheelPowers driveFieldCentric(double forward, double right, double clockwise) {
        return driveFieldCentric(forward, right, clockwise, getYawRadians());
    }

    /** Package-independent overload used by autonomous code and unit tests. */
    public WheelPowers driveFieldCentric(
            double forward, double right, double clockwise, double yawRadians) {
        WheelPowers powers = calculateFieldCentric(
                forward, right, clockwise, yawRadians);
        apply(powers);
        return powers;
    }

    public static WheelPowers calculateFieldCentric(
            double forward, double right, double clockwise, double yawRadians) {
        double cos = Math.cos(yawRadians);
        double sin = Math.sin(yawRadians);
        double robotRight = right * cos + forward * sin;
        double robotForward = -right * sin + forward * cos;
        return calculate(robotForward, robotRight, clockwise);
    }

    public double getYawRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void stop() {
        apply(new WheelPowers(0, 0, 0, 0));
    }

    private void apply(WheelPowers powers) {
        frontLeft.setPower(powers.frontLeft);
        backLeft.setPower(powers.backLeft);
        frontRight.setPower(powers.frontRight);
        backRight.setPower(powers.backRight);
    }

    public static WheelPowers calculate(double forward, double right, double clockwise) {
        double frontLeft = forward + right + clockwise;
        double backLeft = forward - right + clockwise;
        double frontRight = forward - right - clockwise;
        double backRight = forward + right - clockwise;
        double scale = Math.max(1.0, Math.max(Math.abs(frontLeft),
                Math.max(Math.abs(backLeft),
                        Math.max(Math.abs(frontRight), Math.abs(backRight)))));
        return new WheelPowers(
                frontLeft / scale,
                backLeft / scale,
                frontRight / scale,
                backRight / scale);
    }

    public static final class WheelPowers {
        public final double frontLeft;
        public final double backLeft;
        public final double frontRight;
        public final double backRight;

        public WheelPowers(
                double frontLeft, double backLeft, double frontRight, double backRight) {
            this.frontLeft = frontLeft;
            this.backLeft = backLeft;
            this.frontRight = frontRight;
            this.backRight = backRight;
        }
    }
}
