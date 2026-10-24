package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RobotConfig;

/**
 * The shooter, turret and blocker as the autonomous modes use them, set up the same way as
 * Fixed Turret Intake Shooter TeleOp: the turret holds where it starts, and the blocker stays
 * closed except while feeding pollen into the spinning flywheel. The intake drives the
 * transfer, so the caller feeds by running the intake while the blocker is open.
 */
final class AutoShooter {
    private final DcMotor shooter;
    private final DcMotor turret;
    private final Servo blocker;

    AutoShooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotor.class, RobotConfig.SHOOTER_MOTOR);
        turret = hardwareMap.get(DcMotor.class, RobotConfig.TURRET_MOTOR);
        blocker = hardwareMap.get(Servo.class, RobotConfig.BLOCKER_SERVO);
        shooter.setPower(0);
        turret.setPower(0);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // A braking flywheel would jerk the robot and wear the motor; let it spin down.
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Call at START: nothing moves during INIT. */
    void start() {
        turret.setTargetPosition(turret.getCurrentPosition());
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(FixedTurretIntakeShooterTeleOp.TURRET_HOLD_POWER);
        setBlocker(FixedTurretIntakeShooterTeleOp.BLOCKER_BLOCKED_POSITION);
    }

    void spinUp() {
        shooter.setPower(FixedTurretIntakeShooterTeleOp.SHOOTER_POWER);
    }

    void openBlocker() {
        setBlocker(FixedTurretIntakeShooterTeleOp.BLOCKER_CLEAR_POSITION);
    }

    /** Closes the blocker and stops the flywheel; the turret keeps holding. */
    void finishShooting() {
        setBlocker(FixedTurretIntakeShooterTeleOp.BLOCKER_BLOCKED_POSITION);
        shooter.setPower(0);
    }

    void stop() {
        shooter.setPower(0);
        turret.setPower(0);
    }

    private void setBlocker(double position) {
        blocker.setPosition(position);
        if (blocker instanceof ServoImplEx) ((ServoImplEx) blocker).setPwmEnable();
    }
}
