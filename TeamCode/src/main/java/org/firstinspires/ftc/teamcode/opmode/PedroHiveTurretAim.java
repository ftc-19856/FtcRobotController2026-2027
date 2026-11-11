package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedro.BlueAlliancePaths;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
@TeleOp(name = "Pedro Hive Turret Aim", group = "Test")
public class PedroHiveTurretAim extends OpMode {
    private static final Pose BLUE_START = new BlueAlliancePaths().start;
    protected static final double HIVE_X = 83;
    protected static final double HIVE_Y = 83;
    private static final double CALIBRATION_ANGLE_DEGREES = -45;
    private static final double TURRET_POWER = 0.15;

    // Encoder change per degree of counterclockwise turret rotation, including gearing.
    // Measured in INIT: 238 straight ahead, -139 at 90 degrees clockwise.
    public static double TURRET_TICKS_PER_DEGREE = (-139 - 238) / -90.0;

    private PinpointLocalizer localizer;
    private GoBildaPinpointDriver pinpoint;
    private MecanumDrive drive;
    private DcMotor turret;
    private int forwardTicks;
    private double ticksPerDegree;
    private double targetDegrees;
    private boolean poseInitialized;
    private Integer calibrationForwardTicks;
    private boolean previousX;
    private boolean previousY;

    @Override
    public void init() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        drive = new MecanumDrive(hardwareMap);
        turret = hardwareMap.get(DcMotor.class, RobotConfig.TURRET_MOTOR);
        turret.setPower(0);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        poseInitialized = false;
        calibrationForwardTicks = null;
        previousX = false;
        previousY = false;
    }

    @Override
    public void init_loop() {
        localizer.update();
        int encoderTicks = turret.getCurrentPosition();
        if (gamepad1.x && !previousX) calibrationForwardTicks = encoderTicks;
        if (gamepad1.y && !previousY && calibrationForwardTicks != null) {
            TURRET_TICKS_PER_DEGREE = (encoderTicks - (double) calibrationForwardTicks)
                    / CALIBRATION_ANGLE_DEGREES;
        }
        previousX = gamepad1.x;
        previousY = gamepad1.y;
        telemetry.addData("Blue start X / Y / heading (deg)", "%.1f / %.1f / %.1f",
                BLUE_START.x(), BLUE_START.y(), Math.toDegrees(BLUE_START.heading()));
        telemetry.addLine("Point the turret straight ahead before START.");
        telemetry.addLine("Left stick moves the robot. Right stick turns it after START.");
        telemetry.addLine("Optional recalibration: X straight ahead, Y at 45 degrees clockwise.");
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Turret encoder ticks", encoderTicks);
        if (calibrationForwardTicks != null) {
            telemetry.addData("Calibration forward ticks", calibrationForwardTicks);
        }
        telemetry.addData("Calibrated ticks/degree", "%.4f", TURRET_TICKS_PER_DEGREE);
        telemetry.update();
    }

    @Override
    public void start() {
        localizer.update();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) return;
        localizer.setPose(BLUE_START);
        forwardTicks = turret.getCurrentPosition();
        ticksPerDegree = TURRET_TICKS_PER_DEGREE;
        targetDegrees = 0;
        poseInitialized = true;
        if (!Double.isFinite(ticksPerDegree) || ticksPerDegree == 0) return;
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setTargetPosition(forwardTicks);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        localizer.update();
        Pose pose = localizer.pose();
        String status;
        if (!poseInitialized) {
            status = "Restart once Pinpoint shows READY";
        } else if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            status = "Waiting for Pinpoint";
        } else if (!Double.isFinite(ticksPerDegree) || ticksPerDegree == 0) {
            status = "Calibrate turret in INIT before starting";
        } else {
            status = aim(pose);
        }
        if (!status.equals("Tracking")) turret.setPower(0);
        drive.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.addData("Aim", status);
        telemetry.addData("Robot X / Y / heading (deg)", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Turret zero ticks", forwardTicks);
        telemetry.addData("Turret ticks per CCW degree", "%.4f", ticksPerDegree);
        int encoderTicks = turret.getCurrentPosition();
        telemetry.addData("Turret encoder ticks", encoderTicks);
        if (status.equals("Tracking")) {
            afterAimUpdate(pose, targetDegrees, (encoderTicks - (double) forwardTicks) / ticksPerDegree);
        } else {
            afterAimUpdate(pose, Double.NaN, Double.NaN);
        }
        telemetry.update();
    }

    protected void afterAimUpdate(Pose pose, double targetAngle, double actualAngle) { }

    protected Pose aimOrigin(Pose pose) { return pose; }

    private String aim(Pose pose) {
        Pose origin = aimOrigin(pose);
        double dx = HIVE_X - origin.x();
        double dy = HIVE_Y - origin.y();
        double requestedDegrees = Math.toDegrees(Math.atan2(dy, dx) - pose.heading());
        double distance = Math.hypot(dx, dy);
        if (!Double.isFinite(requestedDegrees) || !Double.isFinite(distance)) {
            return "Invalid robot pose";
        }
        if (distance < 1) return "Too close to hive to define aim";

        int currentTicks = turret.getCurrentPosition();
        double currentDegrees = (currentTicks - (double) forwardTicks) / ticksPerDegree;
        // Unwrap the requested bearing using the previous target, independent of encoder lag.
        double nextTargetDegrees = targetDegrees + Math.IEEEremainder(requestedDegrees - targetDegrees, 360);
        double requestedTicks = forwardTicks + nextTargetDegrees * ticksPerDegree;
        if (!Double.isFinite(requestedTicks) || requestedTicks < Integer.MIN_VALUE
                || requestedTicks > Integer.MAX_VALUE) {
            return "Invalid turret target";
        }
        targetDegrees = nextTargetDegrees;
        int targetTicks = (int) Math.round(requestedTicks);
        turret.setTargetPosition(targetTicks);
        turret.setPower(TURRET_POWER);
        telemetry.addData("Turret target / actual (deg)", "%.1f / %.1f", targetDegrees, currentDegrees);
        telemetry.addData("Turret target ticks", targetTicks);
        return "Tracking";
    }

    @Override
    public void stop() {
        if (drive != null) drive.stop();
        if (turret != null) turret.setPower(0);
    }
}
