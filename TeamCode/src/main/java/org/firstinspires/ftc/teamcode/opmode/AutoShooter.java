package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.util.ShooterVelocity;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The shooter, turret and blocker as the autonomous modes use them, set up the same way as
 * Fixed Turret Intake Shooter TeleOp: the turret optionally holds where it starts, and the blocker stays
 * closed except while feeding pollen into the spinning flywheel. The intake drives the
 * transfer, so the caller feeds by running the intake while the blocker is open.
 */
final class AutoShooter {
    private static final double TUNING_FULL_SPEED = 2300;
    private final ShooterVelocity shooter;
    private final DcMotorEx flywheel;
    private final TuningFlywheelControl tuningControl;
    private final double tunedTarget;
    private final ElapsedTime speedTimer = new ElapsedTime();
    private boolean spinning;
    private final DcMotor turret;
    private final Servo blocker;

    AutoShooter(HardwareMap hardwareMap, double tunedSpeedFraction) {
        flywheel = hardwareMap.get(DcMotorEx.class, RobotConfig.SHOOTER_MOTOR);
        tunedTarget = tunedSpeedFraction * TUNING_FULL_SPEED;
        if (tunedTarget > 0) {
            shooter = null;
            tuningControl = new TuningFlywheelControl(TUNING_FULL_SPEED);
            flywheel.setPower(0);
            flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            shooter = new ShooterVelocity(flywheel);
            tuningControl = null;
        }
        turret = hardwareMap.get(DcMotor.class, RobotConfig.TURRET_MOTOR);
        blocker = hardwareMap.get(Servo.class, RobotConfig.BLOCKER_SERVO);
        turret.setPower(0);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Call at START: nothing moves during INIT. */
    void start(boolean holdTurret) {
        if (holdTurret) {
            turret.setTargetPosition(turret.getCurrentPosition());
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(FixedTurretIntakeShooterTeleOp.TURRET_HOLD_POWER);
        } else {
            // No encoder correction or powered turret movement in this mode.
            turret.setPower(0);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        setBlocker(FixedTurretIntakeShooterTeleOp.BLOCKER_BLOCKED_POSITION);
    }

    void spinUp() {
        spinning = true;
        speedTimer.reset();
        if (shooter != null) shooter.setEnabled(true);
    }

    void openBlocker() {
        setBlocker(FixedTurretIntakeShooterTeleOp.BLOCKER_CLEAR_POSITION);
    }

    /** Closes the blocker and stops the flywheel; leaves the turret setting unchanged. */
    void finishShooting() {
        setBlocker(FixedTurretIntakeShooterTeleOp.BLOCKER_BLOCKED_POSITION);
        stopFlywheel();
    }

    void stop() {
        stopFlywheel();
        turret.setPower(0);
    }

    private void setBlocker(double position) {
        blocker.setPosition(position);
        if (blocker instanceof ServoImplEx) ((ServoImplEx) blocker).setPwmEnable();
    }

    private void stopFlywheel() {
        spinning = false;
        if (shooter != null) shooter.stop();
        else {
            tuningControl.power(0, 0, 0);
            flywheel.setPower(0);
        }
    }

    void update() {
        if (shooter != null) shooter.update();
        else if (spinning) {
            double seconds = speedTimer.seconds();
            speedTimer.reset();
            flywheel.setPower(-tuningControl.power(tunedTarget, flywheel.getVelocity(), seconds));
        }
    }

    void addTelemetry(Telemetry telemetry) {
        if (shooter != null) shooter.addTelemetry(telemetry);
        else {
            telemetry.addData("Shooter target / actual (ticks/s)", "%.0f / %.0f",
                    spinning ? tunedTarget : 0, Math.abs(flywheel.getVelocity()));
        }
    }
}
