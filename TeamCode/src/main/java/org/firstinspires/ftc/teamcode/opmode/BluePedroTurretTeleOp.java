package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.Toggle;

@Configurable
@TeleOp(name = "Blue Pedro Turret TeleOp", group = "Competition")
public final class BluePedroTurretTeleOp extends PedroHiveTurretAim {
    private static final double SECOND_HIVE_Y = 55;
    private static final double FULL_SPEED_TICKS_PER_SECOND = 2300;
    // Use this target until field testing supplies a distance-based speed table.
    public static double SHOOTER_TARGET_TICKS_PER_SECOND = 1495;

    private DcMotor intake;
    private DcMotorEx flywheel;
    private Servo blocker;
    private Toggle intakeToggle;
    private Toggle shooterToggle;
    private Toggle secondHive;
    private TuningFlywheelControl flywheelControl;
    private double previousSpeedUpdate;
    private boolean useBlueStart;
    private boolean started;

    @Override
    public void init() {
        super.init();
        useBlueStart = false;
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
        if (gamepad1.dpad_up) useBlueStart = true;
        if (gamepad1.dpad_down) useBlueStart = false;
        telemetry.addData("Pose source", useBlueStart ? "Blue auto start" : "Retained Pinpoint pose");
        telemetry.addData("Retained X / Y / heading (deg)", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addLine("INIT D-pad up: use blue auto start. D-pad down: retain pose.");
    }

    @Override
    protected boolean initializeFieldPose(PinpointLocalizer localizer) {
        if (useBlueStart) return super.initializeFieldPose(localizer);
        Pose pose = localizer.pose();
        return pose.x() > 0 && pose.x() < 144 && pose.y() > 0 && pose.y() < 144
                && Double.isFinite(pose.heading());
    }

    @Override
    public void start() {
        super.start();
        intakeToggle = new Toggle(false, gamepad1.x);
        shooterToggle = new Toggle(false, gamepad1.y);
        secondHive = new Toggle(false, gamepad1.b);
        flywheelControl = new TuningFlywheelControl(FULL_SPEED_TICKS_PER_SECOND);
        previousSpeedUpdate = getRuntime();
        setBlocker(false);
        started = true;
    }

    @Override
    public void loop() {
        secondHive.update(gamepad1.b);
        super.loop();
    }

    @Override
    protected double hiveY() { return secondHive.get() ? SECOND_HIVE_Y : HIVE_Y; }

    @Override
    protected void afterAimUpdate(Pose pose, double targetAngle, double actualAngle) {
        intakeToggle.update(gamepad1.x);
        shooterToggle.update(gamepad1.y);
        intake.setPower(intakeToggle.get() ? FixedTurretIntakeShooterTeleOp.INTAKE_POWER : 0);
        setBlocker(gamepad1.a);

        double configuredSpeed = SHOOTER_TARGET_TICKS_PER_SECOND;
        double targetSpeed = shooterToggle.get() && Double.isFinite(configuredSpeed)
                ? Math.max(0, Math.min(FULL_SPEED_TICKS_PER_SECOND, configuredSpeed)) : 0;
        double encoderSpeed = flywheel.getVelocity();
        double now = getRuntime();
        flywheel.setPower(-flywheelControl.power(targetSpeed, encoderSpeed, now - previousSpeedUpdate));
        previousSpeedUpdate = now;

        telemetry.addData("Hive X / Y", "%.0f / %.0f", hiveX(), hiveY());
        telemetry.addData("Hive distance (in)", "%.1f", Math.hypot(hiveX() - pose.x(), hiveY() - pose.y()));
        telemetry.addData("Shooter target / actual (ticks/s)", "%.0f / %.0f", targetSpeed, Math.abs(encoderSpeed));
        telemetry.addData("Intake", intakeToggle.get() ? "On" : "Off");
        telemetry.addLine("X intake | Y shooter | Hold A blocker open | B switch hive");
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
