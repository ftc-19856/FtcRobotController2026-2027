package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive.WheelPowers;
import org.firstinspires.ftc.teamcode.util.Toggle;

/** Robot-centric driving with toggleable intake/shooter and a fixed turret. */
@TeleOp(name = "Fixed Turret Intake Shooter", group = "Competition")
public final class FixedTurretIntakeShooterTeleOp extends OpMode {
    public static final double INTAKE_POWER = -1.0;
    public static final double SHOOTER_POWER = -1.0;
    public static final double TURRET_HOLD_POWER = 0.2;
    public static final double BLOCKER_BLOCKED_POSITION = 0.155;
    public static final double BLOCKER_CLEAR_POSITION = 0.0;

    private DcMotor intake;
    private MecanumDrive drive;
    private DcMotor shooter;
    private DcMotor turret;
    private Servo blocker;
    private Toggle intakeToggle;
    private Toggle shooterToggle;
    private int turretHoldTicks;
    private boolean running;

    @Override
    public void init() {
        // Intentionally empty for fast Auto -> TeleOp switching. No hardware
        // access, encoder reset, motor power, servo movement, or calibration.
    }

    @Override
    public void start() {
        // Same drivetrain setup as StarterTeleOp, deferred to START so INIT stays empty.
        drive = new MecanumDrive(hardwareMap);
        drive.resetYaw();
        intake = hardwareMap.get(DcMotor.class, RobotConfig.INTAKE_MOTOR);
        shooter = hardwareMap.get(DcMotor.class, RobotConfig.SHOOTER_MOTOR);
        turret = hardwareMap.get(DcMotor.class, RobotConfig.TURRET_MOTOR);
        blocker = hardwareMap.get(Servo.class, RobotConfig.BLOCKER_SERVO);

        intakeToggle = new Toggle(false, gamepad1.x);
        shooterToggle = new Toggle(false, gamepad1.y);

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
        // Intake and shooter start off; X and Y independently toggle them.
        running = true;
    }

    @Override
    public void loop() {
        if (!running) return;
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x;
        WheelPowers powers = drive.driveRobotCentric(forward, right, clockwise);
        intakeToggle.update(gamepad1.x);
        shooterToggle.update(gamepad1.y);
        intake.setPower(intakeToggle.get() ? INTAKE_POWER : 0);
        shooter.setPower(shooterToggle.get() ? SHOOTER_POWER : 0);
        turret.setPower(TURRET_HOLD_POWER);
        if (gamepad1.a) {
            blocker.setPosition(BLOCKER_CLEAR_POSITION);
        } else {
            blocker.setPosition(BLOCKER_BLOCKED_POSITION);
        }
        if (blocker instanceof ServoImplEx) {
            ((ServoImplEx) blocker).setPwmEnable();
        }

        telemetry.addLine("Left stick: move | Right stick: turn (same as Starter TeleOp)");
        telemetry.addLine("Hold A: ready to shoot | Release A: block");
        telemetry.addLine("X: intake toggle | Y: shooter toggle");
        telemetry.addData("Yaw (rad)", "%.3f", drive.getYawRadians());
        telemetry.addData("Drive FL/FR", "%.2f / %.2f", powers.frontLeftMotor, powers.frontRightMotor);
        telemetry.addData("Drive BL/BR", "%.2f / %.2f", powers.backLeft, powers.backRight);
        telemetry.addData("Intake", intakeToggle.get() ? "ON" : "OFF");
        telemetry.addData("Shooter", shooterToggle.get() ? "ON" : "OFF");
        telemetry.addData("Turret hold / actual ticks", "%d / %d",
                turretHoldTicks, turret.getCurrentPosition());
        telemetry.addData("Blocker target", blocker.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        running = false;
        if (drive != null) drive.stop();
        if (intake != null) intake.setPower(0);
        if (shooter != null) shooter.setPower(0);
        if (turret != null) turret.setPower(0);
        if (blocker != null) {
            blocker.setPosition(BLOCKER_BLOCKED_POSITION);
            if (blocker instanceof ServoImplEx) ((ServoImplEx) blocker).setPwmEnable();
        }
    }
}
