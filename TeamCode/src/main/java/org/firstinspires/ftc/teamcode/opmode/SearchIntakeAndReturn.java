package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.localization.PinpointOdometry;

// Spins in place searching with the Limelight ("pollen" pipeline) until it finds a
// target, turns to face and drives up to it while steering off Limelight tx/ta, runs
// the intake, then returns to the start point using the calibrated Pinpoint odometry.
// The Limelight is mounted on the same (front) side as the intake, so reaching "close
// enough" on camera means the ball is already at the intake - no turning around or
// overshoot needed. It's measured 0.4in right of center, so a small strafe-left
// correction brings the ball onto the intake's true centerline.
@Autonomous(name = "Search Intake And Return", group = "Competition")
public final class SearchIntakeAndReturn extends OpMode {
    private static final double SEARCH_SPIN_POWER = 0.25;
    private static final double SEARCH_TIMEOUT_SECONDS = 10.0;

    private static final double TURN_KP = 0.015;
    private static final double MIN_TURN_POWER = 0.1;
    private static final double MAX_TURN_POWER = 0.4;
    // While the target is off by more than this, turn in place only - mixing a large
    // turn with forward power was canceling out on some wheels and barely moving.
    private static final double ALIGN_TOLERANCE_DEGREES = 10.0;
    private static final double APPROACH_FORWARD_POWER = 0.5;
    // Limelight target area (percent of image) at which the ball is considered close
    // enough to intake. Placeholder - tune on the real robot for the camera's mount
    // height/angle and the ball's real size.
    private static final double TARGET_AREA_CLOSE_ENOUGH = 8.0;
    private static final double APPROACH_TIMEOUT_SECONDS = 6.0;
    // If the target is lost from view after its area was at least this fraction of
    // the close-enough threshold, treat it as "got too close for the camera to see"
    // rather than "lost the ball" - common right before contact.
    private static final double LOST_TARGET_ASSUME_ARRIVED_FRACTION = 0.5;

    // Centering the target on camera (tx = 0) aligns it with the camera's sightline,
    // which is 0.4in right of the robot's true centerline - so the ball ends up 0.4in
    // right of the intake's centerline unless corrected. Strafe left by that amount.
    private static final double CAMERA_RIGHT_OFFSET_INCHES = 0.4;
    private static final double STRAFE_CORRECTION_POWER = 0.15;
    private static final double STRAFE_CORRECTION_TIMEOUT_SECONDS = 1.5;

    private static final double POSITION_TOLERANCE_INCHES = 1.5;
    private static final double RETURN_MAX_DRIVE_POWER = 0.5;
    private static final double RETURN_MIN_DRIVE_POWER = 0.15;
    private static final double RETURN_TRANSLATION_KP = 0.03;
    private static final double RETURN_HEADING_KP = 1.5;
    private static final double RETURN_MAX_TURN_POWER = 0.25;
    private static final double RETURN_TIMEOUT_SECONDS = 8.0;

    private static final double INTAKE_DWELL_SECONDS = 1.0;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private MecanumDrive drive;
    private PinpointOdometry odometry;
    private Limelight3A limelight;
    private DcMotorEx intake;
    private State state;
    private double lastSeenTa;
    private double strafeCorrectionStartForward;
    private double strafeCorrectionStartRight;

    private enum State {
        SEARCH,
        APPROACH,
        STRAFE_CORRECTION,
        COLLECT,
        RETURN_TO_START,
        DONE
    }

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        odometry = new PinpointOdometry(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR);
        intake.setPower(0);

        limelight = hardwareMap.get(Limelight3A.class, RobotConfig.LIMELIGHT);
        limelight.pipelineSwitch(RobotConfig.POLLEN_PIPELINE_INDEX);
        limelight.start();

        telemetry.addLine("Initialized; keep still while the Pinpoint IMU calibrates");
        telemetry.update();
    }

    @Override
    public void start() {
        enter(State.SEARCH);
    }

    @Override
    public void loop() {
        odometry.update();
        double forward = odometry.getForwardInches();
        double right = odometry.getRightInches();
        LLResult result = limelight.getLatestResult();
        boolean hasTarget = result != null && result.isValid();
        if (hasTarget) {
            lastSeenTa = result.getTa();
        }

        switch (state) {
            case SEARCH:
                if (hasTarget) {
                    intake.setPower(1);
                    enter(State.APPROACH);
                } else if (timedOut(SEARCH_TIMEOUT_SECONDS)) {
                    telemetry.addLine("Search timed out; no target found");
                    enter(State.DONE);
                } else {
                    drive.driveRobotCentric(0, 0, SEARCH_SPIN_POWER);
                }
                break;

            case APPROACH:
                if (hasTarget && result.getTa() >= TARGET_AREA_CLOSE_ENOUGH) {
                    enterStrafeCorrection(forward, right);
                } else if (hasTarget) {
                    double turn = turnPowerFor(result.getTx());
                    boolean aligned = Math.abs(result.getTx()) <= ALIGN_TOLERANCE_DEGREES;
                    drive.driveRobotCentric(aligned ? APPROACH_FORWARD_POWER : 0, 0, turn);
                    if (timedOut(APPROACH_TIMEOUT_SECONDS)) {
                        enterStrafeCorrection(forward, right);
                    }
                } else if (lastSeenTa >= TARGET_AREA_CLOSE_ENOUGH * LOST_TARGET_ASSUME_ARRIVED_FRACTION) {
                    // Likely just too close for the camera to see anymore.
                    enterStrafeCorrection(forward, right);
                } else if (timedOut(APPROACH_TIMEOUT_SECONDS)) {
                    enter(State.SEARCH);
                } else {
                    drive.stop();
                }
                break;

            case STRAFE_CORRECTION:
                double strafed = Math.hypot(
                        forward - strafeCorrectionStartForward, right - strafeCorrectionStartRight);
                if (strafed >= CAMERA_RIGHT_OFFSET_INCHES || timedOut(STRAFE_CORRECTION_TIMEOUT_SECONDS)) {
                    enter(State.COLLECT);
                } else {
                    drive.driveRobotCentric(0, -STRAFE_CORRECTION_POWER, 0);
                }
                break;

            case COLLECT:
                drive.stop();
                intake.setPower(1);
                if (stateTimer.seconds() >= INTAKE_DWELL_SECONDS) {
                    enter(State.RETURN_TO_START);
                }
                break;

            case RETURN_TO_START:
                intake.setPower(0);
                if (driveToStart(forward, right) || timedOut(RETURN_TIMEOUT_SECONDS)) {
                    enter(State.DONE);
                }
                break;

            case DONE:
                drive.stop();
                intake.setPower(0);
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Has target", hasTarget);
        if (hasTarget) {
            telemetry.addData("tx / ty / ta", "%.1f / %.1f / %.2f",
                    result.getTx(), result.getTy(), result.getTa());
        }
        telemetry.addData("Forward X (in)", "%.2f", forward);
        telemetry.addData("Right Y (in)", "%.2f", right);
        telemetry.update();
    }

    private double turnPowerFor(double txDegrees) {
        double power = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, -txDegrees * TURN_KP));
        if (Math.abs(power) > 0 && Math.abs(power) < MIN_TURN_POWER) {
            power = Math.copySign(MIN_TURN_POWER, power);
        }
        return power;
    }

    // Drives toward the start point (0, 0) using the same odometry feedback approach
    // as IntakeAndReturn. Returns true once within tolerance.
    private boolean driveToStart(double forward, double right) {
        double errorForward = -forward;
        double errorRight = -right;
        double distance = Math.hypot(errorForward, errorRight);
        if (distance <= POSITION_TOLERANCE_INCHES) {
            drive.stop();
            return true;
        }

        double magnitude = clip(distance * RETURN_TRANSLATION_KP,
                RETURN_MIN_DRIVE_POWER, RETURN_MAX_DRIVE_POWER);
        double fieldForward = errorForward / distance * magnitude;
        double fieldRight = errorRight / distance * magnitude;
        double heading = odometry.getHeadingRadians();
        double clockwise = clip(-heading * RETURN_HEADING_KP,
                -RETURN_MAX_TURN_POWER, RETURN_MAX_TURN_POWER);
        drive.driveFieldCentric(fieldForward, fieldRight, clockwise, heading);
        return false;
    }

    private boolean timedOut(double seconds) {
        return stateTimer.seconds() >= seconds;
    }

    private void enter(State nextState) {
        drive.stop();
        state = nextState;
        stateTimer.reset();
    }

    private void enterStrafeCorrection(double forward, double right) {
        strafeCorrectionStartForward = forward;
        strafeCorrectionStartRight = right;
        enter(State.STRAFE_CORRECTION);
    }

    private static double clip(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    @Override
    public void stop() {
        drive.stop();
        intake.setPower(0);
        limelight.stop();
    }
}
