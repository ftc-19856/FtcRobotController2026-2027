package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Driver-controlled program for the 2026-2027 goBILDA StarterBot intake on a
 * Strafer mecanum chassis.
 *
 * <p>Controls:
 * <ul>
 *   <li>Left stick: drive and strafe</li>
 *   <li>Right stick X: turn</li>
 *   <li>A: toggle intake</li>
 *   <li>B: toggle reverse intake</li>
 * </ul>
 *
 * <p>The names passed to {@code hardwareMap.get()} must match the names in the
 * Robot Configuration. Physical hub ports are assigned there, not in Java.
 */
@TeleOp(name = "StarterBot TeleOp", group = "StarterBot")
public class StarterBotTeleOp extends OpMode {
    private static final double INTAKE_POWER = 1.0;

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private double intakePower;
    private boolean previousA;
    private boolean previousB;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        // These directions match goBILDA's mecanum StarterBot example.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // The corner sweepers face each other, so one CR servo must be reversed.
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);

        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopAllMotors();
        intakePower = 0.0;
        previousA = false;
        previousB = false;

        telemetry.addLine("Initialized");
        telemetry.addLine("Left stick: drive/strafe; right stick: turn");
        telemetry.addLine("Tap A: intake; tap B: reverse; tap again: stop");
    }

    @Override
    public void loop() {
        driveMecanum(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x);

        boolean aPressed = gamepad1.a && !previousA;
        boolean bPressed = gamepad1.b && !previousB;

        // Pressing both buttons together stops the intake. Otherwise, tapping
        // the active direction stops it and tapping the other direction switches it.
        if (aPressed && bPressed) {
            intakePower = 0.0;
        } else if (aPressed) {
            intakePower = intakePower == INTAKE_POWER ? 0.0 : INTAKE_POWER;
        } else if (bPressed) {
            intakePower = intakePower == -INTAKE_POWER ? 0.0 : -INTAKE_POWER;
        }

        previousA = gamepad1.a;
        previousB = gamepad1.b;

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        telemetry.addData("Intake", intakePower > 0.0
                ? "IN" : intakePower < 0.0 ? "REVERSE" : "STOPPED");
    }

    @Override
    public void stop() {
        stopAllMotors();
    }

    private void driveMecanum(double forward, double strafe, double turn) {
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);

        double leftFrontPower = (forward + strafe + turn) / denominator;
        double rightFrontPower = (forward - strafe - turn) / denominator;
        double leftBackPower = (forward - strafe + turn) / denominator;
        double rightBackPower = (forward + strafe - turn) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
        intake.setZeroPowerBehavior(behavior);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
        intake.setMode(mode);
    }

    private void stopAllMotors() {
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
        intake.setPower(0.0);
        leftIntakeServo.setPower(0.0);
        rightIntakeServo.setPower(0.0);
    }
}
