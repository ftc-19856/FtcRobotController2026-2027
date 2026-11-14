package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.Toggle;

abstract class AlliancePedroTurretTeleOp extends PedroHiveTurretAim {
    private static final double FULL_SPEED_TICKS_PER_SECOND = TuningFlywheelControl.FULL_SPEED_TICKS_PER_SECOND;

    private final String alliance;
    private final Pose autoStart;
    private final double targetX;
    private final double startingUpY;
    private final double startingDownY;

    private final Gamepad previousSpeedButtons = new Gamepad();
    private DcMotor intake;
    private DcMotorEx flywheel;
    private Servo blocker;
    private Toggle intakeToggle;
    private Toggle shootingToggle;
    private Toggle secondHive;
    private TuningFlywheelControl flywheelControl;
    private double previousSpeedUpdate;
    private double speedTrim;
    private boolean useAutoStart;
    private boolean started;

    AlliancePedroTurretTeleOp(String alliance, Pose autoStart, double targetX,
                              double startingUpY, double startingDownY) {
        this.alliance = alliance;
        this.autoStart = autoStart;
        this.targetX = targetX;
        this.startingUpY = startingUpY;
        this.startingDownY = startingDownY;
    }

    @Override
    public void init() {
        super.init();
        useAutoStart = false;
        started = false;
        intake = hardwareMap.get(DcMotor.class, RobotConfig.INTAKE_MOTOR);
        intake.setPower(0);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel = hardwareMap.get(DcMotorEx.class, RobotConfig.SHOOTER_MOTOR);
        flywheel.setPower(0);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blocker = hardwareMap.get(Servo.class, RobotConfig.BLOCKER_SERVO);
    }

    @Override
    protected PinpointConfig pinpointConfig() { return Constants.retainPinpointPoseConfig(); }

    @Override
    protected void showStartPose(Pose pose) {
        if (gamepad1.dpad_up) useAutoStart = true;
        if (gamepad1.dpad_down) useAutoStart = false;
        telemetry.addData("Pose source", useAutoStart ? alliance + " auto start" : "Retained Pinpoint pose");
        telemetry.addData("Retained X / Y / heading (deg)", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addLine("INIT D-pad up: use " + alliance + " auto start. D-pad down: retain pose.");
    }

    @Override
    protected boolean initializeFieldPose(PinpointLocalizer localizer) {
        if (useAutoStart) {
            localizer.setPose(autoStart);
            return true;
        }
        Pose pose = localizer.pose();
        return pose.x() > 0 && pose.x() < 144 && pose.y() > 0 && pose.y() < 144
                && Double.isFinite(pose.heading());
    }

    @Override
    public void start() {
        super.start();
        intakeToggle = new Toggle(false, gamepad1.x);
        shootingToggle = new Toggle(false, gamepad1.y);
        secondHive = new Toggle(false, gamepad1.b);
        flywheelControl = new TuningFlywheelControl(FULL_SPEED_TICKS_PER_SECOND);
        previousSpeedUpdate = getRuntime();
        speedTrim = 0;
        previousSpeedButtons.copy(gamepad1);
        setBlocker(false);
        started = true;
    }

    @Override
    public void loop() {
        secondHive.update(gamepad1.b);
        shootingToggle.update(gamepad1.y);
        super.loop();
    }

    @Override
    protected boolean turretAimEnabled() { return shootingToggle != null && shootingToggle.get(); }

    @Override
    protected final double hiveX() { return targetX; }

    @Override
    protected final double hiveY() {
        return secondHive != null && secondHive.get() ? startingDownY : startingUpY;
    }

    @Override
    protected void afterAimUpdate(Pose pose, double targetAngle, double actualAngle) {
        intakeToggle.update(gamepad1.x);
        double intakePower = FixedTurretIntakeShooterTeleOp.INTAKE_POWER;
        if (gamepad1.left_bumper) {
            intake.setPower(-intakePower);
        } else {
            intake.setPower(intakeToggle.get() ? intakePower : 0);
        }
        setBlocker(gamepad1.a);

        int speedChangePercent = (gamepad1.dpad_up && !previousSpeedButtons.dpad_up ? 5 : 0)
                - (gamepad1.dpad_down && !previousSpeedButtons.dpad_down ? 5 : 0)
                + (gamepad1.dpad_right && !previousSpeedButtons.dpad_right ? 1 : 0)
                - (gamepad1.dpad_left && !previousSpeedButtons.dpad_left ? 1 : 0);
        previousSpeedButtons.copy(gamepad1);
        Pose origin = aimOrigin(pose);
        double distance = hasFieldPose() ? Math.hypot(hiveX() - origin.x(), hiveY() - origin.y()) : Double.NaN;
        double baseSpeed = TuningFlywheelControl.speedForDistance(distance)
                - FULL_SPEED_TICKS_PER_SECOND * RobotConfig.PEDRO_SHOOTER_REDUCTION_PERCENT / 100.0;
        // Limit trim to ±20 percentage points so the running shooter keeps a positive target.
        double maxTrim = 0.20 * FULL_SPEED_TICKS_PER_SECOND;
        speedTrim = Math.max(-maxTrim, Math.min(maxTrim,
                speedTrim + FULL_SPEED_TICKS_PER_SECOND * speedChangePercent / 100.0));
        double targetSpeed = shootingToggle.get() ? baseSpeed + speedTrim : 0;
        double encoderSpeed = flywheel.getVelocity();
        double now = getRuntime();
        flywheel.setPower(-flywheelControl.power(targetSpeed, encoderSpeed, now - previousSpeedUpdate));
        previousSpeedUpdate = now;

        telemetry.addData("Hive X / Y", "%.0f / %.0f", hiveX(), hiveY());
        telemetry.addData("Hive distance (in)", Double.isFinite(distance)
                ? String.format(java.util.Locale.US, "%.1f", distance) : "Unavailable");
        telemetry.addData("Shooter target / actual (ticks/s)", "%.0f / %.0f", targetSpeed, Math.abs(encoderSpeed));
        telemetry.addData("Shooter base / trim (ticks/s)", "%.0f / %+.0f", baseSpeed, speedTrim);
        telemetry.addData("Intake", gamepad1.left_bumper ? "Reverse" : (intakeToggle.get() ? "On" : "Off"));
        telemetry.addLine("X intake | Hold LB reverse | Hold A blocker open");
        telemetry.addLine("Y toggle shooter and turret aim | B switch hive");
        telemetry.addLine("D-pad up/down: shooter trim +/-5 points. Left/right: +/-1 point.");
    }

    private void setBlocker(boolean open) {
        blocker.setPosition(open ? FixedTurretIntakeShooterTeleOp.BLOCKER_CLEAR_POSITION
                : FixedTurretIntakeShooterTeleOp.BLOCKER_BLOCKED_POSITION);
        if (blocker instanceof ServoImplEx) ((ServoImplEx) blocker).setPwmEnable();
    }

    @Override
    public void stop() {
        if (intake != null) intake.setPower(0);
        if (flywheel != null) flywheel.setPower(0);
        if (started) setBlocker(false);
        super.stop();
    }
}
