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
import org.firstinspires.ftc.teamcode.pedro.BlueAlliancePaths;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
@TeleOp(name = "Pedro Hive Turret Aim", group = "Test")
public final class PedroHiveTurretAim extends OpMode {
    private static final Pose BLUE_START = new BlueAlliancePaths().start;
    private static final double HIVE_X = 83;
    private static final double HIVE_Y = 83;
    private static final double TURRET_RATIO = 2.75;
    private static final double TURRET_POWER = 0.15;

    // Use the FTC motor type by default. Override with ticks per motor output shaft turn.
    public static double MOTOR_TICKS_PER_REV_OVERRIDE = 0;
    public static boolean REVERSE_TURRET_ENCODER = false;

    private PinpointLocalizer localizer;
    private GoBildaPinpointDriver pinpoint;
    private DcMotor turret;
    private int forwardTicks;
    private double ticksPerDegree;
    private boolean poseInitialized;

    @Override
    public void init() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        turret = hardwareMap.get(DcMotor.class, RobotConfig.TURRET_MOTOR);
        turret.setPower(0);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        poseInitialized = false;
    }

    @Override
    public void init_loop() {
        localizer.update();
        telemetry.addData("Blue start X / Y / heading (deg)", "%.1f / %.1f / %.1f",
                BLUE_START.x(), BLUE_START.y(), Math.toDegrees(BLUE_START.heading()));
        telemetry.addLine("Center the turret before START.");
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Turret encoder ticks", turret.getCurrentPosition());
        telemetry.addData("Configured motor ticks/rev", turret.getMotorType().getTicksPerRev());
        telemetry.update();
    }

    @Override
    public void start() {
        localizer.update();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) return;
        localizer.setPose(BLUE_START);
        forwardTicks = turret.getCurrentPosition();
        double motorTicks = MOTOR_TICKS_PER_REV_OVERRIDE == 0
                ? turret.getMotorType().getTicksPerRev() : MOTOR_TICKS_PER_REV_OVERRIDE;
        poseInitialized = true;
        if (!Double.isFinite(motorTicks) || motorTicks <= 0) {
            ticksPerDegree = 0;
            return;
        }
        ticksPerDegree = (motorTicks / 360.0) * TURRET_RATIO;
        if (REVERSE_TURRET_ENCODER) ticksPerDegree = -ticksPerDegree;
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
        } else if (ticksPerDegree == 0) {
            status = "Check motor type and encoder scale, then restart";
        } else {
            status = aim(pose);
        }
        if (!status.equals("Tracking")) turret.setPower(0);
        telemetry.addData("Aim", status);
        telemetry.addData("Robot X / Y / heading (deg)", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Turret encoder ticks", turret.getCurrentPosition());
        telemetry.update();
    }

    private String aim(Pose pose) {
        double dx = HIVE_X - pose.x();
        double dy = HIVE_Y - pose.y();
        double requestedDegrees = Math.toDegrees(Math.atan2(dy, dx) - pose.heading());
        double distance = Math.hypot(dx, dy);
        if (!Double.isFinite(requestedDegrees) || !Double.isFinite(distance)) {
            return "Invalid robot pose";
        }
        if (distance < 1) return "Too close to hive to define aim";

        int currentTicks = turret.getCurrentPosition();
        double currentDegrees = (currentTicks - (double) forwardTicks) / ticksPerDegree;
        // Select the equivalent target nearest the current turret angle across the wrap.
        double correction = Math.IEEEremainder(requestedDegrees - currentDegrees, 360);
        long targetTicks = Math.round(currentTicks + correction * ticksPerDegree);
        if (!Double.isFinite(correction) || targetTicks < Integer.MIN_VALUE || targetTicks > Integer.MAX_VALUE) {
            return "Invalid turret target";
        }
        turret.setTargetPosition((int) targetTicks);
        turret.setPower(TURRET_POWER);
        telemetry.addData("Turret angle / correction (deg)", "%.1f / %.1f", currentDegrees, correction);
        telemetry.addData("Turret target ticks", targetTicks);
        return "Tracking";
    }

    @Override
    public void stop() {
        if (turret != null) turret.setPower(0);
    }
}
