package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.io.IOException;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.util.ShooterVelocity;

/** Records manually scored pollen shots while driving and aiming with Pedro. */
@Configurable
@TeleOp(name = "Pedro Shooter Tuning", group = "Test")
public final class PedroShooterTuning extends PedroHiveTurretAim {
    public static int START_SPEED_PERCENT = 65;
    public static String HOOD_SETTING = "fixed";
    public static double PIVOT_FORWARD_IN = 0;
    public static double PIVOT_LEFT_IN = 0;

    private final Gamepad previous = new Gamepad();
    private DcMotor intake;
    private DcMotorEx flywheel;
    private ShooterVelocity shooter;
    private ShotTuningLog log;
    private int speedPercent;
    private int pollenPerTrial;
    private double pivotForward;
    private double pivotLeft;
    private Trial trial;
    private String status;
    private int savedTrials;

    @Override
    public void init() {
        super.init();
        intake = hardwareMap.get(DcMotor.class, RobotConfig.INTAKE_MOTOR);
        intake.setPower(0);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel = hardwareMap.get(DcMotorEx.class, RobotConfig.SHOOTER_MOTOR);
        shooter = new ShooterVelocity(flywheel);
        log = new ShotTuningLog(hardwareMap.appContext);
    }

    @Override
    public void start() {
        pivotForward = PIVOT_FORWARD_IN;
        pivotLeft = PIVOT_LEFT_IN;
        super.start();
        previous.copy(gamepad1);
        speedPercent = Math.max(1, Math.min(100, START_SPEED_PERCENT));
        pollenPerTrial = 1;
        trial = null;
        savedTrials = 0;
        status = "Drive to a test spot and let the flywheel settle.";
        intake.setPower(FixedTurretIntakeShooterTeleOp.INTAKE_POWER);
        shooter.setEnabled(true, speedPercent / 100.0);
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
            if (gamepad1.left_bumper && !previous.left_bumper) {
                pollenPerTrial = pollenPerTrial == 1 ? 4 : 1;
            }
        }
        shooter.setEnabled(true, speedPercent / 100.0);
        double targetSpeed = Math.abs(shooter.targetTicksPerSecond());
        double actualSpeed = -flywheel.getVelocity();
        Pose origin = aimOrigin(pose);
        double distance = Math.hypot(HIVE_X - origin.x(), HIVE_Y - origin.y());

        if (trial != null && gamepad1.x && !previous.x) {
            trial = null;
            status = "Trial discarded.";
        } else if (trial == null && gamepad1.right_bumper && !previous.right_bumper) {
            if (Double.isFinite(targetAngle) && Double.isFinite(actualAngle)
                    && Double.isFinite(distance) && targetSpeed > 0
                    && Double.isFinite(actualSpeed) && actualSpeed > 0) {
                trial = new Trial(pose, distance, targetAngle, actualAngle, targetSpeed, actualSpeed);
                status = "Recording. Release RB after the pollen leaves.";
            } else {
                status = "Wait for turret tracking and flywheel speed before recording.";
            }
        } else if (trial != null && trial.recording) {
            trial.sample(pose, targetAngle, actualAngle, actualSpeed);
            if (!gamepad1.right_bumper) {
                trial.recording = false;
                trial.durationSeconds = getRuntime() - trial.startSeconds;
                status = "Mark each pollen result.";
            }
        } else if (trial != null) {
            scoreTrial();
        }
        previous.copy(gamepad1);

        telemetry.addData("Hive distance (in)", "%.1f", distance);
        telemetry.addData("Shooter setting", "%d%%", speedPercent);
        shooter.addTelemetry(telemetry);
        telemetry.addData("Shot trial", status);
        telemetry.addData("Saved trials", savedTrials);
        telemetry.addData("Pollen per trial", pollenPerTrial);
        if (trial == null) {
            telemetry.addLine("D-pad up/down changes speed 5%. Left/right changes it 1%.");
            telemetry.addLine("LB selects 1 or 4 pollen. Hold RB to record while feeding.");
        } else if (!trial.recording) {
            telemetry.addData("Scored", "%d / %d", trial.scored(), pollenPerTrial);
            telemetry.addLine("A hit | B miss | D-pad up high | down low | X discard");
        }
    }

    private void scoreTrial() {
        if (gamepad1.a && !previous.a) trial.hits++;
        else if (gamepad1.b && !previous.b) trial.misses++;
        else if (gamepad1.dpad_up && !previous.dpad_up) trial.high++;
        else if (gamepad1.dpad_down && !previous.dpad_down) trial.low++;
        if (trial.scored() < pollenPerTrial) return;
        try {
            log.append(trial.csv());
            savedTrials++;
            status = String.format(Locale.US, "Saved %d/%d hits.", trial.hits, pollenPerTrial);
        } catch (IOException exception) {
            status = "Could not save trial: " + exception.getMessage();
        }
        trial = null;
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
        if (shooter != null) shooter.stop();
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

        private Trial(Pose pose, double distance, double targetAngle, double actualAngle,
                      double targetSpeed, double actualSpeed) {
            startSeconds = getRuntime();
            startPose = pose;
            minimumSpeed = actualSpeed;
            sample(pose, targetAngle, actualAngle, actualSpeed);
            setup = String.format(Locale.US, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.3f,%.3f,%.3f,%s,%.3f,%.3f,%d",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()), distance, targetAngle, actualAngle,
                    speedPercent, targetSpeed, actualSpeed, batteryVolts(), ShotTuningLog.quote(HOOD_SETTING),
                    pivotForward, pivotLeft, pollenPerTrial);
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
            return setup + String.format(Locale.US, ",%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d",
                    minimumSpeed, durationSeconds, movement, headingChange, aimError, hits, misses, high, low);
        }
    }
}
