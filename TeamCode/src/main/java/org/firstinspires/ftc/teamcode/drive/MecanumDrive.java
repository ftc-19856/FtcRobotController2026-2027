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
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backRightMotor;
    private final IMU imu;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.FRONT_LEFT_MOTOR);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.BACK_LEFT_MOTOR);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.FRONT_RIGHT_MOTOR);
        backRightMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.BACK_RIGHT_MOTOR);
        imu = hardwareMap.get(IMU.class, RobotConfig.IMU);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientation));

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        configureMotor(frontLeftMotor);
        configureMotor(backLeftMotor);
        configureMotor(frontRightMotor);
        configureMotor(backRightMotor);
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
        frontLeftMotor.setPower(powers.frontLeftMotor);
        backLeftMotor.setPower(powers.backLeft);
        frontRightMotor.setPower(powers.frontRightMotor);
        backRightMotor.setPower(powers.backRight);
    }

    public static WheelPowers calculate(double forward, double right, double clockwise) {
        double frontLeftMotor = forward + right + clockwise;
        double backLeft = forward - right + clockwise;
        double frontRightMotor = forward - right - clockwise;
        double backRight = forward + right - clockwise;
        double scale = Math.max(1.0, Math.max(Math.abs(frontLeftMotor),
                Math.max(Math.abs(backLeft),
                        Math.max(Math.abs(frontRightMotor), Math.abs(backRight)))));
        return new WheelPowers(
                frontLeftMotor / scale,
                backLeft / scale,
                frontRightMotor / scale,
                backRight / scale);
    }

    public static final class WheelPowers {
        public final double frontLeftMotor;
        public final double backLeft;
        public final double frontRightMotor;
        public final double backRight;

        public WheelPowers(
                double frontLeftMotor, double backLeft, double frontRightMotor, double backRight) {
            this.frontLeftMotor = frontLeftMotor;
            this.backLeft = backLeft;
            this.frontRightMotor = frontRightMotor;
            this.backRight = backRight;
        }
    }
}
