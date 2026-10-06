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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

// Spins in place until the Limelight ("pollen" pipeline) sees the ball, then stops and
// averages a short burst of readings into one exact (forward, right) position for the ball
// in the odometry frame, using the camera's known height and mounting position. After
// that the camera is no longer used: the robot drives to that point with the intake
// leading, intakes, and returns to the start with the calibrated Pinpoint odometry.
// The camera is assumed level (no downward tilt) and facing the robot's driving direction.
@Autonomous(name = "Search Intake And Return", group = "Competition")
public final class SearchIntakeAndReturn extends OpMode {
    private static final String TAG = "SearchIntakeAndReturn";
    private static final double LOG_INTERVAL_SECONDS = 0.25;

    private static final double SEARCH_SPIN_POWER = 0.25;
    private static final double SEARCH_TIMEOUT_SECONDS = 10.0;

    // Geometry, in inches. Forward/right are measured from the robot's center of rotation.
    private static final double CAMERA_HEIGHT_INCHES = 7.75;
    private static final double BALL_CENTER_HEIGHT_INCHES = 2.85 / 2;
    private static final double CAMERA_FORWARD_OFFSET_INCHES = 11.5;
    private static final double CAMERA_RIGHT_OFFSET_INCHES = 0.4;
    private static final double INTAKE_FORWARD_OFFSET_INCHES = 9.25;

    // Ignore detections smaller than this (percent of the image): the Limelight has been
    // reporting constant tiny-area noise (up to about 0.07%) that is not the ball.
    private static final double MIN_TARGET_AREA_PERCENT = 0.1;
    // A standard field is 144in across, so a ball farther than that is a bad reading.
    private static final double MAX_BALL_DISTANCE_INCHES = 144.0;

    private static final double MEASURE_SETTLE_SECONDS = 0.3;
    private static final double MEASURE_WINDOW_SECONDS = 0.5;
    private static final int MIN_SAMPLES = 5;
    // At least this fraction of samples must land within this radius of the median, or
    // the readings are scattered noise and the robot goes back to searching.
    private static final double CONSISTENCY_RADIUS_INCHES = 3.0;
    private static final double MIN_CONSISTENT_FRACTION = 0.6;

    private static final double POSITION_TOLERANCE_INCHES = 1.5;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(5);
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MIN_DRIVE_POWER = 0.15;
    private static final double TRANSLATION_KP = 0.03;
    private static final double HEADING_KP = 1.5;
    private static final double MAX_TURN_POWER = 0.25;
    private static final double GO_TO_BALL_TIMEOUT_SECONDS = 8.0;
    private static final double RETURN_TIMEOUT_SECONDS = 8.0;

    private static final double INTAKE_DWELL_SECONDS = 1.0;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();
    private final List<Double> sampleForwards = new ArrayList<>();
    private final List<Double> sampleRights = new ArrayList<>();
    private MecanumDrive drive;
    private PinpointOdometry odometry;
    private Limelight3A limelight;
    private DcMotorEx intake;
    private State state;
    private double ballForward;
    private double ballRight;
    private double targetForward;
    private double targetRight;
    private double targetHeading;

    private enum State {
        SEARCH,
        MEASURE,
        GO_TO_BALL,
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
        double heading = odometry.getHeadingRadians();
        LLResult result = limelight.getLatestResult();
        double[] ball = ballPosition(result, forward, right, heading);

        if (logTimer.seconds() >= LOG_INTERVAL_SECONDS) {
            logTimer.reset();
            logSnapshot(result, ball, forward, right, heading);
        }

        switch (state) {
            case SEARCH:
                if (ball != null) {
                    sampleForwards.clear();
                    sampleRights.clear();
                    enter(State.MEASURE);
                } else if (timedOut(SEARCH_TIMEOUT_SECONDS)) {
                    Log.i(TAG, "search timed out; no ball found");
                    enter(State.DONE);
                } else {
                    drive.driveRobotCentric(0, 0, SEARCH_SPIN_POWER);
                }
                break;

            case MEASURE:
                if (ball != null && stateTimer.seconds() >= MEASURE_SETTLE_SECONDS) {
                    sampleForwards.add(ball[0]);
                    sampleRights.add(ball[1]);
                }
                if (timedOut(MEASURE_SETTLE_SECONDS + MEASURE_WINDOW_SECONDS)) {
                    finishMeasurement(forward, right, heading);
                }
                break;

            case GO_TO_BALL:
                if (driveToPoint(targetForward, targetRight, targetHeading,
                        forward, right, heading) || timedOut(GO_TO_BALL_TIMEOUT_SECONDS)) {
                    enter(State.COLLECT);
                }
                break;

            case COLLECT:
                drive.stop();
                intake.setPower(1);
                if (timedOut(INTAKE_DWELL_SECONDS)) {
                    enter(State.RETURN_TO_START);
                }
                break;

            case RETURN_TO_START:
                intake.setPower(0);
                if (driveToPoint(0, 0, 0, forward, right, heading)
                        || timedOut(RETURN_TIMEOUT_SECONDS)) {
                    enter(State.DONE);
                }
                break;

            case DONE:
                drive.stop();
                intake.setPower(0);
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Ball (forward, right)", "%.1f, %.1f", ballForward, ballRight);
        telemetry.addData("Forward X (in)", "%.2f", forward);
        telemetry.addData("Right Y (in)", "%.2f", right);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.update();
    }

    // Turns the averaged samples into the ball's position and the point the robot center
    // must reach so the intake's pickup point lands on the ball, then heads there.
    private void finishMeasurement(double forward, double right, double heading) {
        int count = sampleForwards.size();
        if (count < MIN_SAMPLES) {
            Log.i(TAG, "measure failed: only " + count + " valid samples");
            enter(State.SEARCH);
            return;
        }

        double medianForward = median(sampleForwards);
        double medianRight = median(sampleRights);
        int consistent = 0;
        for (int i = 0; i < count; i++) {
            if (Math.hypot(sampleForwards.get(i) - medianForward,
                    sampleRights.get(i) - medianRight) <= CONSISTENCY_RADIUS_INCHES) {
                consistent++;
            }
        }
        if (consistent < MIN_CONSISTENT_FRACTION * count) {
            Log.i(TAG, "measure failed: only " + consistent + " of " + count
                    + " samples agree");
            enter(State.SEARCH);
            return;
        }

        ballForward = medianForward;
        ballRight = medianRight;

        double toBallForward = ballForward - forward;
        double toBallRight = ballRight - right;
        double distance = Math.hypot(toBallForward, toBallRight);
        if (distance > 1e-6) {
            double travel = Math.max(distance - INTAKE_FORWARD_OFFSET_INCHES, 0);
            targetForward = forward + toBallForward / distance * travel;
            targetRight = right + toBallRight / distance * travel;
            // Facing direction for heading h is (cos h, -sin h) in (forward, right).
            targetHeading = Math.atan2(-toBallRight, toBallForward);
        } else {
            targetForward = forward;
            targetRight = right;
            targetHeading = heading;
        }

        Log.i(TAG, String.format(
                "ball at forward=%.1f right=%.1f from %d samples (%d agree); "
                        + "target forward=%.1f right=%.1f heading=%.1fdeg",
                ballForward, ballRight, count, consistent,
                targetForward, targetRight, Math.toDegrees(targetHeading)));
        intake.setPower(1);
        enter(State.GO_TO_BALL);
    }

    // Returns the ball's position in the odometry frame (forward, right), or null if the
    // reading isn't a trustworthy detection of the ball.
    private static double[] ballPosition(
            LLResult result, double forward, double right, double heading) {
        if (result == null || !result.isValid() || result.getTa() < MIN_TARGET_AREA_PERCENT) {
            return null;
        }
        return ballFieldPosition(result.getTx(), result.getTy(), forward, right, heading);
    }

    // Converts the Limelight angles to the ball's field position, or null if the angles
    // can't be a ball on the floor within the field. With a level camera the ball center
    // sits below the lens, so ty must be negative and the distance ahead of the camera is
    // (camera height - ball center height) / tan(-ty).
    static double[] ballFieldPosition(
            double txDegrees, double tyDegrees, double forward, double right, double heading) {
        if (tyDegrees >= 0) {
            return null;
        }
        double ahead = (CAMERA_HEIGHT_INCHES - BALL_CENTER_HEIGHT_INCHES)
                / Math.tan(Math.toRadians(-tyDegrees));
        if (ahead > MAX_BALL_DISTANCE_INCHES) {
            return null;
        }
        double robotForward = CAMERA_FORWARD_OFFSET_INCHES + ahead;
        double robotRight = CAMERA_RIGHT_OFFSET_INCHES
                + ahead * Math.tan(Math.toRadians(txDegrees));
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return new double[] {
                forward + robotForward * cos + robotRight * sin,
                right - robotForward * sin + robotRight * cos
        };
    }

    // Drives to a field point while turning to the target heading. Returns true once the
    // robot is within tolerance of both.
    private boolean driveToPoint(double goalForward, double goalRight, double goalHeading,
            double forward, double right, double heading) {
        double errorForward = goalForward - forward;
        double errorRight = goalRight - right;
        double distance = Math.hypot(errorForward, errorRight);
        double headingError = normalizeAngle(heading - goalHeading);
        boolean atPoint = distance <= POSITION_TOLERANCE_INCHES;
        if (atPoint && Math.abs(headingError) <= HEADING_TOLERANCE_RADIANS) {
            drive.stop();
            return true;
        }

        double fieldForward = 0;
        double fieldRight = 0;
        if (!atPoint) {
            double magnitude = clip(distance * TRANSLATION_KP, MIN_DRIVE_POWER, MAX_DRIVE_POWER);
            fieldForward = errorForward / distance * magnitude;
            fieldRight = errorRight / distance * magnitude;
        }
        double clockwise = holdHeadingTurn(heading, goalHeading, HEADING_KP, MAX_TURN_POWER);
        drive.driveFieldCentric(fieldForward, fieldRight, clockwise, heading);
        return false;
    }

    // Returns the clockwise turn power that brings heading back to the target. The
    // Pinpoint's heading increases counter-clockwise while the drive's "clockwise" input
    // is clockwise-positive, so the error is heading minus target; the opposite sign
    // pushes the heading further away and the robot spins in circles.
    static double holdHeadingTurn(
            double heading, double targetHeading, double kp, double maxTurn) {
        return clip(normalizeAngle(heading - targetHeading) * kp, -maxTurn, maxTurn);
    }

    private void logSnapshot(
            LLResult result, double[] ball, double forward, double right, double heading) {
        String limelightPart = result != null && result.isValid()
                ? String.format("tx=%.1f ty=%.1f ta=%.2f",
                        result.getTx(), result.getTy(), result.getTa())
                : "no detection";
        String ballPart = ball != null
                ? String.format("ball=(%.1f, %.1f)", ball[0], ball[1])
                : "ball=none";
        Log.i(TAG, String.format("state=%s %s %s heading=%.1fdeg forward=%.2f right=%.2f",
                state, limelightPart, ballPart, Math.toDegrees(heading), forward, right));
    }

    private boolean timedOut(double seconds) {
        return stateTimer.seconds() >= seconds;
    }

    private void enter(State nextState) {
        Log.i(TAG, String.format("%s -> %s", state, nextState));
        drive.stop();
        state = nextState;
        stateTimer.reset();
    }

    private static double median(List<Double> values) {
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        int n = sorted.size();
        return n % 2 == 1 ? sorted.get(n / 2) : (sorted.get(n / 2 - 1) + sorted.get(n / 2)) / 2;
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
