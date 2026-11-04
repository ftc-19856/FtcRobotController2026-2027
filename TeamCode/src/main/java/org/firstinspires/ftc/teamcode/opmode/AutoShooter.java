package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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
    private final ShooterVelocity shooter;
    private final DcMotor turret;
    private final Servo blocker;

    AutoShooter(HardwareMap hardwareMap) {
        shooter = new ShooterVelocity(hardwareMap.get(DcMotorEx.class, RobotConfig.SHOOTER_MOTOR));
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
        shooter.setEnabled(true);
    }

    void openBlocker() {
        setBlocker(FixedTurretIntakeShooterTeleOp.BLOCKER_CLEAR_POSITION);
    }

    /** Closes the blocker and stops the flywheel; leaves the turret setting unchanged. */
    void finishShooting() {
        setBlocker(FixedTurretIntakeShooterTeleOp.BLOCKER_BLOCKED_POSITION);
        shooter.stop();
    }

    void stop() {
        shooter.stop();
        turret.setPower(0);
    }

    private void setBlocker(double position) {
        blocker.setPosition(position);
        if (blocker instanceof ServoImplEx) ((ServoImplEx) blocker).setPwmEnable();
    }

    void update() { shooter.update(); }

    void addTelemetry(Telemetry telemetry) { shooter.addTelemetry(telemetry); }
}
