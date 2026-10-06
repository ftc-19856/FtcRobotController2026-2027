package org.firstinspires.ftc.teamcode.opmode;

import android.util.Log;

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
// target, turns in place to align with it, then LOCKS that heading and drives straight
// on odometry heading-hold rather than continuing to steer off live (and noisy) tx
// every loop - runs the intake, then returns to the start point using the calibrated
// Pinpoint odometry. The Limelight is mounted on the same (front) side as the intake,
// so the robot drives until the ball leaves the camera's view, at which point the ball
// is already at the intake - no turning around or overshoot needed. The camera is
// measured 0.4in right of center, so a small strafe-left correction brings the ball
// onto the intake's true centerline.
@Autonomous(name = "Search Intake And Return", group = "Competition")
public final class SearchIntakeAndReturn extends OpMode {
    private static final String TAG = "SearchIntakeAndReturn";
    // How often to log tx/ty/ta/heading while in a state - logging every loop would
    // flood logcat, this gives enough resolution to reconstruct what happened.
    private static final double LOG_INTERVAL_SECONDS = 0.25;

    private static final double SEARCH_SPIN_POWER = 0.25;
    private static final double SEARCH_TIMEOUT_SECONDS = 10.0;

    private static final double TURN_KP = 0.015;
    private static final double MIN_TURN_POWER = 0.1;
    private static final double MAX_TURN_POWER = 0.4;
    // How close tx needs to be to call the robot aligned and lock in a heading.
    private static final double ALIGN_TOLERANCE_DEGREES = 5.0;
    private static final double ALIGN_TIMEOUT_SECONDS = 3.0;

    // Once aligned, drive straight on the locked heading instead of continuing to
    // steer off live tx every loop - constantly re-steering off a noisy tx reading
    // was fighting itself and the robot was barely moving forward at all.
    private static final double DRIVE_FORWARD_POWER = 0.5;
    private static final double DRIVE_HEADING_KP = 1.5;
    private static final double DRIVE_MAX_TURN_POWER = 0.25;
    private static final double DRIVE_TIMEOUT_SECONDS = 6.0;
    // Keep driving until the ball has been out of the camera's view for this long. The
    // short delay ignores single-frame dropouts; it also sets how far past the point
    // where the ball leaves view the robot travels, so tune it if it stops short or long.
    private static final double LOST_TARGET_SECONDS = 0.2;

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
    private final ElapsedTime logTimer = new ElapsedTime();
    private final ElapsedTime targetLostTimer = new ElapsedTime();
    private MecanumDrive drive;
    private PinpointOdometry odometry;
    private Limelight3A limelight;
    private DcMotorEx intake;
    private State state;
    private double lockedHeadingRadians;
    private double strafeCorrectionStartForward;
    private double strafeCorrectionStartRight;

    private enum State {
        SEARCH,
        ALIGN,
        DRIVE,
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
        // The Pinpoint's forward axis currently reads negative when the robot drives
        // forward, so flip it into the drive's convention. Remove this if the forward
        // pod direction is ever reversed in PinpointOdometry.
        double forward = -odometry.getForwardInches();
        double right = odometry.getRightInches();
        LLResult result = limelight.getLatestResult();
        boolean hasTarget = result != null && result.isValid();
        if (hasTarget) {
            targetLostTimer.reset();
        }

        if (logTimer.seconds() >= LOG_INTERVAL_SECONDS) {
            logTimer.reset();
            if (hasTarget) {
                Log.i(TAG, String.format(
                        "state=%s hasTarget=true tx=%.1f ty=%.1f ta=%.2f heading=%.1fdeg forward=%.2f right=%.2f",
                        state, result.getTx(), result.getTy(), result.getTa(),
                        Math.toDegrees(odometry.getHeadingRadians()), forward, right));
            } else {
                Log.i(TAG, String.format(
                        "state=%s hasTarget=false heading=%.1fdeg forward=%.2f right=%.2f",
                        state, Math.toDegrees(odometry.getHeadingRadians()), forward, right));
            }
        }

        switch (state) {
            case SEARCH:
                if (hasTarget) {
                    intake.setPower(1);
                    enter(State.ALIGN);
                } else if (timedOut(SEARCH_TIMEOUT_SECONDS)) {
                    telemetry.addLine("Search timed out; no target found");
                    enter(State.DONE);
                } else {
                    drive.driveRobotCentric(0, 0, SEARCH_SPIN_POWER);
                }
                break;

            case ALIGN:
                if (hasTarget && Math.abs(result.getTx()) <= ALIGN_TOLERANCE_DEGREES) {
                    lockedHeadingRadians = odometry.getHeadingRadians();
                    enter(State.DRIVE);
                } else if (hasTarget) {
                    drive.driveRobotCentric(0, 0, turnPowerFor(result.getTx()));
                    if (timedOut(ALIGN_TIMEOUT_SECONDS)) {
                        // Close enough - lock in whatever heading we've got and go.
                        lockedHeadingRadians = odometry.getHeadingRadians();
                        enter(State.DRIVE);
                    }
                } else if (timedOut(ALIGN_TIMEOUT_SECONDS)) {
                    enter(State.SEARCH);
                } else {
                    drive.stop();
                }
                break;

            case DRIVE:
                if (!hasTarget && targetLostTimer.seconds() >= LOST_TARGET_SECONDS) {
                    // Ball is out of view - it's under the camera/at the intake now.
                    enterStrafeCorrection(forward, right);
                } else if (timedOut(DRIVE_TIMEOUT_SECONDS)) {
                    enterStrafeCorrection(forward, right);
                } else {
                    double heading = odometry.getHeadingRadians();
                    drive.driveRobotCentric(DRIVE_FORWARD_POWER, 0,
                            holdHeadingTurn(heading, lockedHeadingRadians,
                                    DRIVE_HEADING_KP, DRIVE_MAX_TURN_POWER));
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
        double clockwise = holdHeadingTurn(heading, 0, RETURN_HEADING_KP, RETURN_MAX_TURN_POWER);
        drive.driveFieldCentric(fieldForward, fieldRight, clockwise, heading);
        return false;
    }

    // Returns the clockwise turn power that brings heading back to the target. The
    // Pinpoint's heading increases counter-clockwise while the drive's "clockwise" input
    // is clockwise-positive, so the error is heading minus target; the opposite sign
    // pushes the heading further away and the robot spins in circles.
    private static double holdHeadingTurn(
            double heading, double targetHeading, double kp, double maxTurn) {
        return clip(normalizeAngle(heading - targetHeading) * kp, -maxTurn, maxTurn);
    }

    private boolean timedOut(double seconds) {
        return stateTimer.seconds() >= seconds;
    }

    private void enter(State nextState) {
        Log.i(TAG, String.format("%s -> %s heading=%.1fdeg",
                state, nextState, Math.toDegrees(odometry.getHeadingRadians())));
        drive.stop();
        state = nextState;
        stateTimer.reset();
    }

    private void enterStrafeCorrection(double forward, double right) {
        strafeCorrectionStartForward = forward;
        strafeCorrectionStartRight = right;
        enter(State.STRAFE_CORRECTION);
    }

    private static double normalizeAngle(double radians) {
        double normalized = radians;
        while (normalized > Math.PI) {
            normalized -= 2 * Math.PI;
        }
        while (normalized <= -Math.PI) {
            normalized += 2 * Math.PI;
        }
        return normalized;
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
