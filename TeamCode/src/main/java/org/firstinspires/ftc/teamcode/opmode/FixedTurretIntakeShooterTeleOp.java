package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.revhub.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Robot-centric driving with continuous intake/shooter and a fixed turret. */
@TeleOp(name = "Fixed Turret Intake Shooter", group = "Competition")
public final class FixedTurretIntakeShooterTeleOp extends OpMode {
    public static final double INTAKE_POWER = -1.0;
    public static final double SHOOTER_POWER = -1.0;
    public static final double TURRET_HOLD_POWER = 0.2;
    public static final double BLOCKER_POSITION_OFFSET = 0.2;
    public static final double SLOW_DRIVE_SCALE = 0.35;
    private static final double STICK_DEADBAND = 0.05;

    private DcMotor intake;
    private Drivetrain drive;
    private DcMotor shooter;
    private DcMotor turret;
    private Servo blocker;
    private RelativeBlockerToggle blockerToggle;
    private int turretHoldTicks;
    private boolean running;

    @Override
    public void init() {
        // Intentionally empty for fast Auto -> TeleOp switching. No hardware
        // access, encoder reset, motor power, servo movement, or calibration.
    }

    @Override
    public void start() {
        // Reuse Pedro's actual motor names/directions. Robot-centric drive does
        // not need an IMU/Pinpoint initialization or any autonomous pose reset.
        drive = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        drive.stop(true);
        intake = hardwareMap.get(DcMotor.class, RobotConfig.INTAKE_MOTOR);
        shooter = hardwareMap.get(DcMotor.class, RobotConfig.SHOOTER_MOTOR);
        turret = hardwareMap.get(DcMotor.class, RobotConfig.TURRET_MOTOR);
        blocker = hardwareMap.get(Servo.class, RobotConfig.BLOCKER_SERVO);

        // Capture the last servo command without moving it. This is not an
        // absolute angle measurement and is not inferred from a mechanical stop.
        blockerToggle = new RelativeBlockerToggle(blocker.getPosition(), gamepad1.a, gamepad1.b);

        intake.setPower(0);
        shooter.setPower(0);
        turret.setPower(0);
        // Match the -1 intake / -1 shooter directions in the user's working test.
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretHoldTicks = turret.getCurrentPosition();
        turret.setTargetPosition(turretHoldTicks);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(TURRET_HOLD_POWER);
        intake.setPower(INTAKE_POWER);
        shooter.setPower(SHOOTER_POWER);
        running = true;
    }

    @Override
    public void loop() {
        if (!running) return;
        drive.drive(drivingPowers(gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x, gamepad1.left_bumper), true);
        intake.setPower(INTAKE_POWER);
        shooter.setPower(SHOOTER_POWER);
        turret.setPower(TURRET_HOLD_POWER);
        double command = blockerToggle.update(gamepad1.a, gamepad1.b);
        if (Double.isFinite(command)) blocker.setPosition(command);

        telemetry.addLine("Left stick: move | Right stick: turn | Left bumper: slow drive");
        telemetry.addLine("A: toggle blocker | B: reverse blocker direction at starting position");
        telemetry.addData("Drive mode", gamepad1.left_bumper ? "ROBOT CENTRIC - SLOW" : "ROBOT CENTRIC");
        telemetry.addData("Intake / shooter power", "%.1f / %.1f", INTAKE_POWER, SHOOTER_POWER);
        telemetry.addData("Turret hold / actual ticks", "%d / %d",
                turretHoldTicks, turret.getCurrentPosition());
        telemetry.addData("Blocker state", blockerToggle.offsetActive ? "OFFSET POSITION" : "START POSITION");
        telemetry.addData("Blocker starting command", blockerToggle.startPosition);
        telemetry.addData("Blocker offset command", blockerToggle.offsetPosition());
        telemetry.addData("Toggle direction", blockerToggle.direction > 0 ? "+0.2 position" : "-0.2 position");
        if (!validPosition(blockerToggle.startPosition)) {
            telemetry.addLine("No known starting servo command. Toggle disabled; no position was guessed.");
        } else if (!validPosition(blockerToggle.offsetPosition())) {
            telemetry.addLine("0.2 offset exceeds servo range. Press B to reverse direction.");
        }
        telemetry.update();
    }

    static boolean validPosition(double position) {
        return Double.isFinite(position) && position >= 0 && position <= 1;
    }

    static DrivePowers drivingPowers(double leftX, double leftY, double rightX, boolean slow) {
        // Pedro uses forward, left, counter-clockwise. Gamepad Y is down-positive;
        // stick X is right-positive, hence all three sign conversions below.
        double forward = stick(-leftY);
        double left = stick(-leftX);
        double counterClockwise = stick(-rightX);
        // Normalize before scaling so combined full-stick inputs remain capped
        // at 35% wheel power in slow mode rather than being normalized back to 100%.
        double denominator = Math.max(1, Math.abs(forward) + Math.abs(left) + Math.abs(counterClockwise));
        double scale = (slow ? SLOW_DRIVE_SCALE : 1) / denominator;
        return new DrivePowers(forward * scale, left * scale, counterClockwise * scale);
    }

    private static double stick(double value) {
        if (!Double.isFinite(value) || Math.abs(value) < STICK_DEADBAND) return 0;
        return Math.max(-1, Math.min(1, value));
    }

    static final class RelativeBlockerToggle {
        final double startPosition;
        boolean offsetActive;
        int direction = 1;
        private boolean previousA;
        private boolean previousB;

        RelativeBlockerToggle(double startPosition, boolean aHeld, boolean bHeld) {
            this.startPosition = startPosition;
            previousA = aHeld;
            previousB = bHeld;
        }

        double offsetPosition() {
            return startPosition + direction * BLOCKER_POSITION_OFFSET;
        }

        double update(boolean a, boolean b) {
            boolean reverse = b && !previousB;
            boolean toggle = a && !previousA;
            previousA = a;
            previousB = b;
            // Changing direction while offset would unexpectedly cross the
            // starting position. Require a return to the starting position first.
            if (reverse && !offsetActive) direction = -direction;
            if (!toggle || b || !validPosition(startPosition)) return Double.NaN;
            double next = offsetActive ? startPosition : offsetPosition();
            // Never clip an out-of-range command into a shorter, misleading move.
            if (!validPosition(next)) return Double.NaN;
            offsetActive = !offsetActive;
            return next;
        }
    }

    @Override
    public void stop() {
        running = false;
        if (drive != null) drive.stop(true);
        if (intake != null) intake.setPower(0);
        if (shooter != null) shooter.setPower(0);
        if (turret != null) turret.setPower(0);
        // Leave the blocker at its last selected position; do not guess open/closed.
    }
}
