package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.io.IOException;
import java.util.Locale;
import java.util.StringJoiner;
import org.firstinspires.ftc.teamcode.RobotConfig;

/** Records manually scored pollen shots while driving and aiming with Pedro. */
@Configurable
@TeleOp(name = "Pedro Shooter Tuning", group = "Test")
public final class PedroShooterTuning extends PedroHiveTurretAim {
    private static final int POLLEN_PER_BURST = 4;
    public static int START_SPEED_PERCENT = 65;
    // Starting estimate from the approximately 2260–2300 ticks/s seen at saturation.
    public static double FULL_SPEED_TICKS_PER_SECOND = 2300;
    public static String HOOD_SETTING = "fixed";
    public static double PIVOT_FORWARD_IN = 0;
    public static double PIVOT_LEFT_IN = 0;

    private final Gamepad previous = new Gamepad();
    private DcMotor intake;
    private DcMotorEx flywheel;
    private Servo blocker;
    private boolean blockerOpen;
    private boolean started;
    private TuningFlywheelControl flywheelControl;
    private ShotTuningLog log;
    private int speedPercent;
    private double fullSpeed;
    private double previousSpeedUpdate;
    private double pivotForward;
    private double pivotLeft;
    private Trial trial;
    private String status;
    private int savedTrials;

    @Override
    public void init() {
        super.init();
        started = false;
        blocker = hardwareMap.get(Servo.class, RobotConfig.BLOCKER_SERVO);
        intake = hardwareMap.get(DcMotor.class, RobotConfig.INTAKE_MOTOR);
        intake.setPower(0);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel = hardwareMap.get(DcMotorEx.class, RobotConfig.SHOOTER_MOTOR);
        flywheel.setPower(0);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        log = new ShotTuningLog(hardwareMap.appContext);
    }

    @Override
    public void start() {
        pivotForward = PIVOT_FORWARD_IN;
        pivotLeft = PIVOT_LEFT_IN;
        super.start();
        previous.copy(gamepad1);
        speedPercent = Math.max(1, Math.min(100, START_SPEED_PERCENT));
        fullSpeed = FULL_SPEED_TICKS_PER_SECOND;
        if (!Double.isFinite(fullSpeed) || fullSpeed <= 0) fullSpeed = 2300;
        flywheelControl = new TuningFlywheelControl(fullSpeed);
        previousSpeedUpdate = getRuntime();
        trial = null;
        savedTrials = 0;
        status = "Drive to a test spot and let the flywheel settle.";
        setBlocker(false);
        started = true;
        flywheel.setPower(-speedPercent / 100.0);
    }

    @Override
    protected Pose aimOrigin(Pose pose) {
        double cos = Math.cos(pose.heading());
        double sin = Math.sin(pose.heading());
        return new Pose(pose.x() + pivotForward * cos - pivotLeft * sin,
                pose.y() + pivotForward * sin + pivotLeft * cos, pose.heading());
    }

    @Override
    protected void afterAimUpdate(Pose pose, double targetAngle, double actualAngle) {
        if (trial == null) {
            int change = (gamepad1.dpad_up && !previous.dpad_up ? 5 : 0)
                    - (gamepad1.dpad_down && !previous.dpad_down ? 5 : 0)
                    + (gamepad1.dpad_right && !previous.dpad_right ? 1 : 0)
                    - (gamepad1.dpad_left && !previous.dpad_left ? 1 : 0);
            speedPercent = Math.max(1, Math.min(100, speedPercent + change));
        }
        double now = getRuntime();
        double targetSpeed = fullSpeed * (speedPercent / 100.0);
        double encoderVelocity = flywheel.getVelocity();
        double actualSpeed = Math.abs(encoderVelocity);
        double power = flywheelControl.power(targetSpeed, encoderVelocity, now - previousSpeedUpdate);
        previousSpeedUpdate = now;
        flywheel.setPower(-power);
        Pose origin = aimOrigin(pose);
        double distance = Math.hypot(HIVE_X - origin.x(), HIVE_Y - origin.y());
        String readiness = recordingReadiness(targetAngle, actualAngle, distance, targetSpeed, actualSpeed);

        if (gamepad1.left_bumper != blockerOpen) setBlocker(gamepad1.left_bumper);
        boolean intakeOn = gamepad1.right_trigger > 0.5;
        intake.setPower(intakeOn ? FixedTurretIntakeShooterTeleOp.INTAKE_POWER : 0);
        if (trial != null && gamepad1.x && !previous.x) {
            trial = null;
            status = "Trial discarded.";
        } else if (trial == null && gamepad1.right_bumper && !previous.right_bumper) {
            trial = new Trial(pose, distance, targetAngle, actualAngle, targetSpeed, actualSpeed);
            status = "Recording burst. Release RB after the fourth pollen.";
        } else if (trial != null && trial.recording) {
            trial.sample(pose, targetAngle, actualAngle, actualSpeed);
            if (!gamepad1.right_bumper) {
                trial.recording = false;
                trial.durationSeconds = getRuntime() - trial.startSeconds;
                status = "Mark four results in firing order.";
                scoreTrial();
            }
        } else if (trial != null) {
            scoreTrial();
        }
        previous.copy(gamepad1);

        telemetry.addData("Hive distance (in)", "%.1f", distance);
        telemetry.addData("Shooter setting", "%d%%", speedPercent);
        telemetry.addData("Shooter target / actual (ticks/s)", "%.0f / %.0f", targetSpeed, actualSpeed);
        telemetry.addData("Shooter signed encoder / motor power", "%.0f / %.2f", encoderVelocity, -power);
        telemetry.addData("Shot trial", status);
        telemetry.addData("Shot readings", readiness);
        telemetry.addData("Saved trials", savedTrials);
        telemetry.addData("Blocker (hold LB)", blockerOpen ? "Open" : "Closed");
        telemetry.addData("Intake (hold right trigger)", intakeOn ? "On" : "Off");
        if (trial == null) {
            telemetry.addLine("D-pad up/down changes speed 5%. Left/right changes it 1%.");
            telemetry.addLine("Hold RB to record four pollen.");
        } else if (!trial.recording) {
            telemetry.addData("Burst results", "%d / %d  %s", trial.scored(), POLLEN_PER_BURST, trial.sequence);
            telemetry.addLine("A hit | B miss | D-pad up high | down low | X discard");
        }
    }

    private static String recordingReadiness(double targetAngle, double actualAngle, double distance,
                                             double targetSpeed, double actualSpeed) {
        if (!Double.isFinite(distance)) return "Pedro distance unavailable";
        if (!Double.isFinite(targetAngle) || !Double.isFinite(actualAngle)) return "Turret tracking unavailable";
        if (targetSpeed <= 0) return "Shooter target is zero";
        if (!Double.isFinite(actualSpeed) || actualSpeed <= 0) return "No flywheel encoder speed";
        return "Pose, turret, and shooter readings available";
    }

    private void scoreTrial() {
        if (gamepad1.a && !previous.a) {
            trial.hits++;
            trial.sequence.add("hit");
        } else if (gamepad1.b && !previous.b) {
            trial.misses++;
            trial.sequence.add("miss");
        } else if (gamepad1.dpad_up && !previous.dpad_up) {
            trial.high++;
            trial.sequence.add("high");
        } else if (gamepad1.dpad_down && !previous.dpad_down) {
            trial.low++;
            trial.sequence.add("low");
        }
        if (trial.scored() < POLLEN_PER_BURST) return;
        try {
            log.append(trial.csv());
            savedTrials++;
            status = "Saved burst: " + trial.sequence;
        } catch (IOException exception) {
            status = "Could not save trial: " + exception.getMessage();
        }
        trial = null;
    }

    private void setBlocker(boolean open) {
        blocker.setPosition(open ? FixedTurretIntakeShooterTeleOp.BLOCKER_CLEAR_POSITION
                : FixedTurretIntakeShooterTeleOp.BLOCKER_BLOCKED_POSITION);
        if (blocker instanceof ServoImplEx) ((ServoImplEx) blocker).setPwmEnable();
        blockerOpen = open;
    }

    private double batteryVolts() {
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (Double.isFinite(voltage) && voltage > 0) return voltage;
        }
        return Double.NaN;
    }

    @Override
    public void stop() {
        if (intake != null) intake.setPower(0);
        if (flywheel != null) flywheel.setPower(0);
        if (started) setBlocker(false);
        super.stop();
    }

    private final class Trial {
        private final String setup;
        private final Pose startPose;
        private final double startSeconds;
        private boolean recording = true;
        private double minimumSpeed;
        private double durationSeconds;
        private double movement;
        private double headingChange;
        private double aimError;
        private int hits;
        private int misses;
        private int high;
        private int low;
        private final StringJoiner sequence = new StringJoiner(" ");

        private Trial(Pose pose, double distance, double targetAngle, double actualAngle,
                      double targetSpeed, double actualSpeed) {
            startSeconds = getRuntime();
            startPose = pose;
            minimumSpeed = actualSpeed;
            sample(pose, targetAngle, actualAngle, actualSpeed);
            setup = String.format(Locale.US, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.3f,%.3f,%.3f,%s,%.3f,%.3f,%d",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()), distance, targetAngle, actualAngle,
                    speedPercent, targetSpeed, actualSpeed, batteryVolts(), ShotTuningLog.quote(HOOD_SETTING),
                    pivotForward, pivotLeft, POLLEN_PER_BURST);
        }

        private int scored() { return hits + misses + high + low; }

        private void sample(Pose pose, double targetAngle, double actualAngle, double actualSpeed) {
            minimumSpeed = Math.min(minimumSpeed, actualSpeed);
            movement = Math.max(movement, Math.hypot(pose.x() - startPose.x(), pose.y() - startPose.y()));
            headingChange = Math.max(headingChange, Math.abs(Math.toDegrees(
                    Math.IEEEremainder(pose.heading() - startPose.heading(), 2 * Math.PI))));
            aimError = Math.max(aimError, Math.abs(targetAngle - actualAngle));
        }

        private String csv() {
            return setup + String.format(Locale.US, ",%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%s",
                    minimumSpeed, durationSeconds, movement, headingChange, aimError, hits, misses, high, low,
                    sequence);
        }
    }
}
