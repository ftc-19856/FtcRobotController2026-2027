package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotConfig;

/** Continuous intake/shooter, encoder-held turret, relative blocker toggle. */
@TeleOp(name = "Fixed Turret Intake Shooter", group = "Test")
public final class FixedTurretIntakeShooterTeleOp extends OpMode {
    public static final double INTAKE_POWER = -1.0;
    public static final double SHOOTER_POWER = -1.0;
    public static final double TURRET_HOLD_POWER = 0.2;
    public static final double BLOCKER_POSITION_OFFSET = 0.2;

    private DcMotor intake;
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
        intake.setPower(INTAKE_POWER);
        shooter.setPower(SHOOTER_POWER);
        turret.setPower(TURRET_HOLD_POWER);
        double command = blockerToggle.update(gamepad1.a, gamepad1.b);
        if (Double.isFinite(command)) blocker.setPosition(command);

        telemetry.addLine("A: toggle blocker | B: reverse movement direction at starting position");
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
        if (intake != null) intake.setPower(0);
        if (shooter != null) shooter.setPower(0);
        if (turret != null) turret.setPower(0);
        // Leave the blocker at its last selected position; do not guess open/closed.
    }
}
