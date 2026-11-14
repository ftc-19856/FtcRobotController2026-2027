package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
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
    private static final double MAX_TURRET_DEGREES = 90;
    private static final String HIVE_OUT_OF_RANGE = "Hive outside turret range. Turn robot to aim.";

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
    private boolean atTravelLimit;
    private boolean poseInitialized;
    private String poseStartStatus;
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
        PinpointConfig config = pinpointConfig();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, config.name.get());
        localizer = new PinpointLocalizer(hardwareMap, config);
        poseInitialized = false;
        poseStartStatus = "Restart once Pinpoint shows READY";
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
        showStartPose(localizer.pose());
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
        if (!initializeFieldPose(localizer)) {
            poseStartStatus = "Field pose unavailable. Select a known start pose in INIT.";
            return;
        }
        forwardTicks = turret.getCurrentPosition();
        ticksPerDegree = TURRET_TICKS_PER_DEGREE;
        targetDegrees = 0;
        atTravelLimit = false;
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
            status = poseStartStatus;
        } else if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            status = "Waiting for Pinpoint";
        } else if (!Double.isFinite(ticksPerDegree) || ticksPerDegree == 0) {
            status = "Calibrate turret in INIT before starting";
        } else {
            status = aim(pose);
        }
        if (!status.equals("Tracking") && !status.equals(HIVE_OUT_OF_RANGE)) {
            turret.setPower(0);
        }
        drive.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.addData("Aim", status);
        telemetry.addData("Robot X / Y / heading (deg)", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Turret zero ticks", forwardTicks);
        telemetry.addData("Turret ticks per CCW degree", "%.4f", ticksPerDegree);
        int encoderTicks = turret.getCurrentPosition();
        telemetry.addData("Turret encoder ticks", encoderTicks);
        telemetry.addData("Turret limits (deg from center)", "CW %.0f / CCW %.0f",
                MAX_TURRET_DEGREES, MAX_TURRET_DEGREES);
        if (poseInitialized && Double.isFinite(ticksPerDegree) && ticksPerDegree != 0) {
            double angle = (encoderTicks - (double) forwardTicks) / ticksPerDegree;
            double limit = Math.floor(MAX_TURRET_DEGREES * Math.abs(ticksPerDegree)) / Math.abs(ticksPerDegree);
            telemetry.addData("Turret travel left (deg)", "CW %.1f / CCW %.1f",
                    Math.max(0, limit + angle), Math.max(0, limit - angle));
        }
        if (status.equals("Tracking")) {
            afterAimUpdate(pose, targetDegrees, (encoderTicks - (double) forwardTicks) / ticksPerDegree);
        } else {
            afterAimUpdate(pose, Double.NaN, Double.NaN);
        }
        telemetry.update();
    }

    protected void afterAimUpdate(Pose pose, double targetAngle, double actualAngle) { }

    protected PinpointConfig pinpointConfig() { return Constants.localizerConfig; }

    protected boolean initializeFieldPose(PinpointLocalizer localizer) {
        localizer.setPose(BLUE_START);
        return true;
    }

    protected void showStartPose(Pose pose) {
        telemetry.addData("Blue start X / Y / heading (deg)", "%.1f / %.1f / %.1f",
                BLUE_START.x(), BLUE_START.y(), Math.toDegrees(BLUE_START.heading()));
    }

    protected Pose aimOrigin(Pose pose) { return pose; }

    protected boolean hasFieldPose() {
        return poseInitialized && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    protected double hiveX() { return HIVE_X; }

    protected double hiveY() { return HIVE_Y; }

    private String aim(Pose pose) {
        Pose origin = aimOrigin(pose);
        double targetX = hiveX();
        double targetY = hiveY();
        if (!Double.isFinite(targetX) || !Double.isFinite(targetY)) return "Hive position unavailable";
        double dx = targetX - origin.x();
        double dy = targetY - origin.y();
        double requestedDegrees = Math.toDegrees(Math.atan2(dy, dx) - pose.heading());
        double distance = Math.hypot(dx, dy);
        if (!Double.isFinite(requestedDegrees) || !Double.isFinite(distance)) {
            return "Invalid robot pose";
        }
        if (distance < 1) return "Too close to hive to define aim";

        int currentTicks = turret.getCurrentPosition();
        double currentDegrees = (currentTicks - (double) forwardTicks) / ticksPerDegree;
        double bearing = Math.IEEEremainder(requestedDegrees, 360);
        boolean outsideRange = Math.abs(bearing) > MAX_TURRET_DEGREES;
        // Stay on the same stop if an out-of-range bearing crosses the rear of the robot.
        double nextTargetDegrees = outsideRange
                ? Math.copySign(MAX_TURRET_DEGREES, atTravelLimit ? targetDegrees : bearing) : bearing;
        double requestedTicks = forwardTicks + limitedTickOffset(nextTargetDegrees, ticksPerDegree);
        if (!Double.isFinite(requestedTicks) || requestedTicks < Integer.MIN_VALUE
                || requestedTicks > Integer.MAX_VALUE) {
            return "Invalid turret target";
        }
        targetDegrees = nextTargetDegrees;
        atTravelLimit = outsideRange;
        int targetTicks = (int) Math.round(requestedTicks);
        turret.setTargetPosition(targetTicks);
        turret.setPower(TURRET_POWER);
        telemetry.addData("Turret target / actual (deg)", "%.1f / %.1f", targetDegrees, currentDegrees);
        telemetry.addData("Turret target ticks", targetTicks);
        return outsideRange ? HIVE_OUT_OF_RANGE : "Tracking";
    }

    static double limitedTickOffset(double degrees, double ticksPerDegree) {
        double maxTicks = Math.floor(MAX_TURRET_DEGREES * Math.abs(ticksPerDegree));
        return Math.max(-maxTicks, Math.min(maxTicks, Math.round(degrees * ticksPerDegree)));
    }

    @Override
    public void stop() {
        if (drive != null) drive.stop();
        if (turret != null) turret.setPower(0);
    }
}
