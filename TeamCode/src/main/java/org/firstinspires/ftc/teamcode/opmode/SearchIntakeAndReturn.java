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

// Starts in the back-left corner of an 8ft (96in) square, facing into it, and stays inside
// it: moves out of the corner, sweeps the camera across the square (a quarter turn), and if
// that finds nothing, searches again from the middle. Limelight readings are turned into one
// field position for the ball; far balls get a second, closer measurement. Then it drives
// there by odometry with the intake leading, intakes, and returns to the start. Every drive
// target is clamped so the whole robot stays in the square, and it stops if odometry ever
// says it's outside. The camera is assumed level and facing the robot's driving direction.
@Autonomous(name = "Search Intake And Return", group = "Competition")
public final class SearchIntakeAndReturn extends OpMode {
    private static final String TAG = "SearchIntakeAndReturn";
    private static final double LOG_INTERVAL_SECONDS = 0.25;

    // Robot footprint, in inches from its center of rotation. The body is 20in long and
    // 17.5in wide; the camera sticks out to 11.5in in front of center.
    static final double ROBOT_FRONT_INCHES = 11.5;
    static final double ROBOT_BACK_INCHES = 10.0;
    static final double ROBOT_HALF_WIDTH_INCHES = 17.5 / 2;
    private static final double[][] FOOTPRINT_CORNERS = {
            {ROBOT_FRONT_INCHES, ROBOT_HALF_WIDTH_INCHES},
            {ROBOT_FRONT_INCHES, -ROBOT_HALF_WIDTH_INCHES},
            {-ROBOT_BACK_INCHES, ROBOT_HALF_WIDTH_INCHES},
            {-ROBOT_BACK_INCHES, -ROBOT_HALF_WIDTH_INCHES}
    };

    // The square in the odometry frame, where the robot's starting center is (0, 0). It
    // starts with its back against the back edge and its left side against the left edge.
    static final double SQUARE_SIZE_INCHES = 96.0;
    static final double SQUARE_MIN_FORWARD = -ROBOT_BACK_INCHES;
    static final double SQUARE_MAX_FORWARD = SQUARE_MIN_FORWARD + SQUARE_SIZE_INCHES;
    static final double SQUARE_MIN_RIGHT = -ROBOT_HALF_WIDTH_INCHES;
    static final double SQUARE_MAX_RIGHT = SQUARE_MIN_RIGHT + SQUARE_SIZE_INCHES;
    // Drive targets keep the robot this far inside the edges.
    static final double BOUNDARY_MARGIN_INCHES = 2.0;
    // Stop if odometry says part of the robot is this far outside for this long. The delay
    // keeps a single glitched reading from ending the run.
    private static final double BOUNDARY_STOP_INCHES = 3.0;
    private static final double BOUNDARY_STOP_SECONDS = 0.15;
    // The robot's farthest point from its center. It can only turn in place without
    // leaving the square when its center is at least this far (plus margin) from every edge.
    static final double TURN_RADIUS_INCHES =
            Math.hypot(Math.max(ROBOT_FRONT_INCHES, ROBOT_BACK_INCHES), ROBOT_HALF_WIDTH_INCHES);
    static final double TURN_SAFE_FORWARD =
            SQUARE_MIN_FORWARD + TURN_RADIUS_INCHES + BOUNDARY_MARGIN_INCHES;
    static final double TURN_SAFE_RIGHT =
            SQUARE_MIN_RIGHT + TURN_RADIUS_INCHES + BOUNDARY_MARGIN_INCHES;
    private static final double CENTER_FORWARD = SQUARE_MIN_FORWARD + SQUARE_SIZE_INCHES / 2;
    private static final double CENTER_RIGHT = SQUARE_MIN_RIGHT + SQUARE_SIZE_INCHES / 2;

    private static final double SEARCH_SPIN_POWER = 0.25;
    // From the corner the whole square lies between straight ahead (0) and directly right
    // (-90 degrees). The camera sees about 27 degrees either side, so -80 covers it.
    private static final double CORNER_SWEEP_END_RADIANS = Math.toRadians(-80);
    private static final double CORNER_SWEEP_TIMEOUT_SECONDS = 5.0;
    private static final double CENTER_SEARCH_TIMEOUT_SECONDS = 10.0;

    // Camera and ball geometry, in inches. Forward/right are from the robot's center.
    private static final double CAMERA_HEIGHT_INCHES = 7.75;
    private static final double BALL_DIAMETER_INCHES = 2.85;
    private static final double BALL_CENTER_HEIGHT_INCHES = BALL_DIAMETER_INCHES / 2;
    private static final double CAMERA_FORWARD_OFFSET_INCHES = 11.5;
    private static final double CAMERA_RIGHT_OFFSET_INCHES = 0.4;
    private static final double INTAKE_FORWARD_OFFSET_INCHES = 9.25;
    // Drive this much past the computed pickup point. On the robot it stopped about 10in
    // short when measuring from about 45in away; the camera underestimates distance by an
    // amount that grows with range, which is why far balls get re-measured from about there.
    private static final double EXTRA_APPROACH_INCHES = 10.0;

    // Ignore detections smaller than this (percent of the image): the Limelight reports
    // constant tiny-area noise (up to about 0.07%) that is not the ball.
    private static final double MIN_TARGET_AREA_PERCENT = 0.1;
    // A standard field is 144in across, so a ball farther than that is a bad reading.
    private static final double MAX_BALL_DISTANCE_INCHES = 144.0;
    // Limelight 3A horizontal field of view; with its 4:3 image this sets how big the ball
    // should look at a given range.
    private static final double LIMELIGHT_HORIZONTAL_FOV_DEGREES = 54.5;
    // Real ball detections have measured 30-50% of the predicted size (the pipeline doesn't
    // pick up the whole ball). A stationary speck that fooled the robot measured under 10%.
    private static final double MIN_AREA_RATIO = 0.2;

    private static final double MEASURE_SETTLE_SECONDS = 0.3;
    private static final double MEASURE_WINDOW_SECONDS = 0.5;
    private static final int MIN_SAMPLES = 5;
    // At least this fraction of samples must land within this radius of the median, or
    // the readings are scattered noise and the robot goes back to searching.
    private static final double CONSISTENCY_RADIUS_INCHES = 3.0;
    private static final double MIN_CONSISTENT_FRACTION = 0.6;
    // A ball farther than this gets a second measurement from this far away.
    private static final double REMEASURE_IF_FARTHER_THAN_INCHES = 60.0;
    private static final double REMEASURE_STANDOFF_INCHES = 44.0;

    private static final double POSITION_TOLERANCE_INCHES = 1.5;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(5);
    // When heading toward the ball, turn in place first if it's off by more than this, so
    // the robot isn't turning while near an edge.
    private static final double TURN_FIRST_TOLERANCE_RADIANS = Math.toRadians(10);
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MIN_DRIVE_POWER = 0.15;
    private static final double TRANSLATION_KP = 0.03;
    private static final double HEADING_KP = 1.5;
    private static final double MAX_TURN_POWER = 0.25;
    private static final double LEAVE_CORNER_TIMEOUT_SECONDS = 3.0;
    private static final double GO_TO_CENTER_TIMEOUT_SECONDS = 6.0;
    private static final double APPROACH_TIMEOUT_SECONDS = 6.0;
    private static final double GO_TO_BALL_TIMEOUT_SECONDS = 8.0;
    private static final double RETURN_TIMEOUT_SECONDS = 8.0;

    private static final double INTAKE_DWELL_SECONDS = 1.0;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();
    private final ElapsedTime insideTimer = new ElapsedTime();
    private final List<Double> sampleForwards = new ArrayList<>();
    private final List<Double> sampleRights = new ArrayList<>();
    private final List<Double> sampleAreas = new ArrayList<>();
    private MecanumDrive drive;
    private PinpointOdometry odometry;
    private Limelight3A limelight;
    private DcMotorEx intake;
    private State state;
    private State measuringFrom;
    private double[] firstBall;
    private double ballForward;
    private double ballRight;
    private double targetForward;
    private double targetRight;
    private double targetHeading;
    private double holdHeading;

    private enum State {
        LEAVE_CORNER,
        SEARCH_CORNER,
        GO_TO_CENTER,
        SEARCH_CENTER,
        MEASURE,
        APPROACH,
        GO_TO_BALL,
        COLLECT,
        RETURN_TO_TURN_SAFE,
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
        insideTimer.reset();
        enter(State.LEAVE_CORNER);
    }

    @Override
    public void loop() {
        odometry.update();
        // The Pinpoint's forward axis reads negative when the robot drives forward, so flip
        // it into the drive's convention. Remove this if the forward pod direction is ever
        // reversed in PinpointOdometry.
        double forward = -odometry.getForwardInches();
        double right = odometry.getRightInches();
        double heading = odometry.getHeadingRadians();
        LLResult result = limelight.getLatestResult();
        double[] ball = ballPosition(result, forward, right, heading);

        if (logTimer.seconds() >= LOG_INTERVAL_SECONDS) {
            logTimer.reset();
            logSnapshot(result, ball, forward, right, heading);
        }

        double overflow = footprintOverflow(forward, right, heading);
        if (overflow <= BOUNDARY_STOP_INCHES) {
            insideTimer.reset();
        } else if (state != State.DONE && insideTimer.seconds() >= BOUNDARY_STOP_SECONDS) {
            Log.i(TAG, String.format("stopping: robot is %.1fin outside the square", overflow));
            enter(State.DONE);
        }

        switch (state) {
            case LEAVE_CORNER:
                if (driveToPoint(TURN_SAFE_FORWARD, TURN_SAFE_RIGHT, 0,
                        forward, right, heading, false)
                        || timedOut(LEAVE_CORNER_TIMEOUT_SECONDS)) {
                    enter(State.SEARCH_CORNER);
                }
                break;

            case SEARCH_CORNER:
                if (ball != null) {
                    startMeasuring(State.SEARCH_CORNER);
                } else if (heading <= CORNER_SWEEP_END_RADIANS
                        || timedOut(CORNER_SWEEP_TIMEOUT_SECONDS)) {
                    holdHeading = heading;
                    enter(State.GO_TO_CENTER);
                } else {
                    holdPositionWhileTurning(TURN_SAFE_FORWARD, TURN_SAFE_RIGHT,
                            SEARCH_SPIN_POWER, forward, right, heading);
                }
                break;

            case GO_TO_CENTER:
                if (driveToPoint(CENTER_FORWARD, CENTER_RIGHT, holdHeading,
                        forward, right, heading, false)
                        || timedOut(GO_TO_CENTER_TIMEOUT_SECONDS)) {
                    enter(State.SEARCH_CENTER);
                }
                break;

            case SEARCH_CENTER:
                if (ball != null) {
                    startMeasuring(State.SEARCH_CENTER);
                } else if (timedOut(CENTER_SEARCH_TIMEOUT_SECONDS)) {
                    Log.i(TAG, "search timed out; no ball found");
                    startReturn(heading);
                } else {
                    holdPositionWhileTurning(CENTER_FORWARD, CENTER_RIGHT,
                            SEARCH_SPIN_POWER, forward, right, heading);
                }
                break;

            case MEASURE:
                if (ball != null && stateTimer.seconds() >= MEASURE_SETTLE_SECONDS) {
                    sampleForwards.add(ball[0]);
                    sampleRights.add(ball[1]);
                    sampleAreas.add(result.getTa());
                }
                if (timedOut(MEASURE_SETTLE_SECONDS + MEASURE_WINDOW_SECONDS)) {
                    finishMeasurement(forward, right, heading);
                }
                break;

            case APPROACH:
                if (driveToPoint(targetForward, targetRight, targetHeading,
                        forward, right, heading, true)
                        || timedOut(APPROACH_TIMEOUT_SECONDS)) {
                    startMeasuring(State.APPROACH);
                }
                break;

            case GO_TO_BALL:
                if (driveToPoint(targetForward, targetRight, targetHeading,
                        forward, right, heading, true)
                        || timedOut(GO_TO_BALL_TIMEOUT_SECONDS)) {
                    enter(State.COLLECT);
                }
                break;

            case COLLECT:
                drive.stop();
                intake.setPower(1);
                if (timedOut(INTAKE_DWELL_SECONDS)) {
                    startReturn(heading);
                }
                break;

            case RETURN_TO_TURN_SAFE:
                intake.setPower(0);
                // Keep the current heading until there's room to turn, then face forward.
                double returnHeading = canTurnInPlace(forward, right) ? 0 : holdHeading;
                if (driveToPoint(TURN_SAFE_FORWARD, TURN_SAFE_RIGHT, returnHeading,
                        forward, right, heading, false)
                        || timedOut(RETURN_TIMEOUT_SECONDS)) {
                    enter(State.RETURN_TO_START);
                }
                break;

            case RETURN_TO_START:
                intake.setPower(0);
                if (driveToPoint(0, 0, 0, forward, right, heading, false)
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

    private void startMeasuring(State from) {
        measuringFrom = from;
        sampleForwards.clear();
        sampleRights.clear();
        sampleAreas.clear();
        enter(State.MEASURE);
    }

    private void startReturn(double heading) {
        holdHeading = heading;
        intake.setPower(0);
        enter(State.RETURN_TO_TURN_SAFE);
    }

    private void finishMeasurement(double forward, double right, double heading) {
        double[] measured = agreedBallPosition();
        if (measured == null) {
            if (measuringFrom == State.APPROACH) {
                Log.i(TAG, "close measurement failed; using the first one");
                goToBall(firstBall, forward, right, heading);
            } else {
                enter(measuringFrom);
            }
            return;
        }

        ballForward = measured[0];
        ballRight = measured[1];
        double distance = Math.hypot(measured[0] - forward, measured[1] - right);
        if (measuringFrom != State.APPROACH && distance > REMEASURE_IF_FARTHER_THAN_INCHES) {
            firstBall = measured;
            setTargetToward(measured, forward, right, heading, REMEASURE_STANDOFF_INCHES);
            enter(State.APPROACH);
        } else {
            goToBall(measured, forward, right, heading);
        }
    }

    private void goToBall(double[] ball, double forward, double right, double heading) {
        setTargetToward(ball, forward, right, heading,
                INTAKE_FORWARD_OFFSET_INCHES - EXTRA_APPROACH_INCHES);
        intake.setPower(1);
        enter(State.GO_TO_BALL);
    }

    // Aims at the point `standoff` inches short of the ball along the line from the robot,
    // facing the ball, then pulls that point back inside the square if needed.
    private void setTargetToward(
            double[] ball, double forward, double right, double heading, double standoff) {
        double toBallForward = ball[0] - forward;
        double toBallRight = ball[1] - right;
        double distance = Math.hypot(toBallForward, toBallRight);
        double goalForward = forward;
        double goalRight = right;
        targetHeading = heading;
        if (distance > 1e-6) {
            double travel = Math.max(distance - standoff, 0);
            goalForward = forward + toBallForward / distance * travel;
            goalRight = right + toBallRight / distance * travel;
            // Facing direction for heading h is (cos h, -sin h) in (forward, right).
            targetHeading = Math.atan2(-toBallRight, toBallForward);
        }
        double[] clamped = clampIntoSquare(goalForward, goalRight, targetHeading);
        targetForward = clamped[0];
        targetRight = clamped[1];
        Log.i(TAG, String.format(
                "target forward=%.1f right=%.1f heading=%.1fdeg (unclamped %.1f, %.1f)",
                targetForward, targetRight, Math.toDegrees(targetHeading),
                goalForward, goalRight));
    }

    // Returns the median ball position, or null (with a logged reason) if there aren't
    // enough samples or they don't agree.
    private double[] agreedBallPosition() {
        int count = sampleForwards.size();
        if (count < MIN_SAMPLES) {
            Log.i(TAG, "measure failed: only " + count + " valid samples");
            return null;
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
            return null;
        }
        Log.i(TAG, String.format(
                "ball at forward=%.1f right=%.1f from %d samples (%d agree), ta %.2f-%.2f",
                medianForward, medianRight, count, consistent,
                Collections.min(sampleAreas), Collections.max(sampleAreas)));
        return new double[] {medianForward, medianRight};
    }

    // Returns the ball's position in the odometry frame (forward, right), or null if the
    // reading isn't a trustworthy detection of a ball inside the square.
    private static double[] ballPosition(
            LLResult result, double forward, double right, double heading) {
        if (result == null || !result.isValid() || result.getTa() < MIN_TARGET_AREA_PERCENT
                || !ballSizePlausible(result.getTx(), result.getTy(), result.getTa())) {
            return null;
        }
        double[] ball = ballFieldPosition(result.getTx(), result.getTy(), forward, right, heading);
        return ball != null && ballInsideSquare(ball[0], ball[1]) ? ball : null;
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
        return toField(forward, right, heading,
                CAMERA_FORWARD_OFFSET_INCHES + ahead,
                CAMERA_RIGHT_OFFSET_INCHES + ahead * Math.tan(Math.toRadians(txDegrees)));
    }

    // True if the detection is about as big as a ball at the distance its angles imply.
    static boolean ballSizePlausible(double txDegrees, double tyDegrees, double areaPercent) {
        if (tyDegrees >= 0) {
            return false;
        }
        double drop = CAMERA_HEIGHT_INCHES - BALL_CENTER_HEIGHT_INCHES;
        double ahead = drop / Math.tan(Math.toRadians(-tyDegrees));
        double lateral = ahead * Math.tan(Math.toRadians(txDegrees));
        double range = Math.sqrt(ahead * ahead + lateral * lateral + drop * drop);
        return areaPercent >= MIN_AREA_RATIO * expectedBallAreaPercent(range);
    }

    // Percent of a 4:3 Limelight image a ball fills at this distance from the lens.
    static double expectedBallAreaPercent(double rangeInches) {
        double focalOverWidth =
                0.5 / Math.tan(Math.toRadians(LIMELIGHT_HORIZONTAL_FOV_DEGREES / 2));
        double diameterOverWidth = BALL_DIAMETER_INCHES / rangeInches * focalOverWidth;
        return 100 * Math.PI / 4 * diameterOverWidth * diameterOverWidth * 4 / 3;
    }

    static boolean ballInsideSquare(double forward, double right) {
        return forward >= SQUARE_MIN_FORWARD && forward <= SQUARE_MAX_FORWARD
                && right >= SQUARE_MIN_RIGHT && right <= SQUARE_MAX_RIGHT;
    }

    // How far the robot's footprint sticks outside the square, or 0 if it's inside.
    static double footprintOverflow(double forward, double right, double heading) {
        double worst = 0;
        for (double[] corner : FOOTPRINT_CORNERS) {
            double[] point = toField(forward, right, heading, corner[0], corner[1]);
            worst = Math.max(worst, Math.max(
                    Math.max(SQUARE_MIN_FORWARD - point[0], point[0] - SQUARE_MAX_FORWARD),
                    Math.max(SQUARE_MIN_RIGHT - point[1], point[1] - SQUARE_MAX_RIGHT)));
        }
        return worst;
    }

    // Moves a target center point so the robot, at the given heading, sits at least the
    // margin inside every edge of the square.
    static double[] clampIntoSquare(double forward, double right, double heading) {
        double minForward = Double.POSITIVE_INFINITY;
        double maxForward = Double.NEGATIVE_INFINITY;
        double minRight = Double.POSITIVE_INFINITY;
        double maxRight = Double.NEGATIVE_INFINITY;
        for (double[] corner : FOOTPRINT_CORNERS) {
            double[] offset = toField(0, 0, heading, corner[0], corner[1]);
            minForward = Math.min(minForward, offset[0]);
            maxForward = Math.max(maxForward, offset[0]);
            minRight = Math.min(minRight, offset[1]);
            maxRight = Math.max(maxRight, offset[1]);
        }
        return new double[] {
                clampRange(forward,
                        SQUARE_MIN_FORWARD + BOUNDARY_MARGIN_INCHES - minForward,
                        SQUARE_MAX_FORWARD - BOUNDARY_MARGIN_INCHES - maxForward),
                clampRange(right,
                        SQUARE_MIN_RIGHT + BOUNDARY_MARGIN_INCHES - minRight,
                        SQUARE_MAX_RIGHT - BOUNDARY_MARGIN_INCHES - maxRight)
        };
    }

    private static boolean canTurnInPlace(double forward, double right) {
        double slack = POSITION_TOLERANCE_INCHES;
        return forward >= TURN_SAFE_FORWARD - slack
                && forward <= SQUARE_MAX_FORWARD - (TURN_SAFE_FORWARD - SQUARE_MIN_FORWARD) + slack
                && right >= TURN_SAFE_RIGHT - slack
                && right <= SQUARE_MAX_RIGHT - (TURN_SAFE_RIGHT - SQUARE_MIN_RIGHT) + slack;
    }

    // Converts a point given relative to the robot (forward, right) into the odometry frame.
    // Heading is counter-clockwise positive.
    private static double[] toField(double forward, double right, double heading,
            double robotForward, double robotRight) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return new double[] {
                forward + robotForward * cos + robotRight * sin,
                right - robotForward * sin + robotRight * cos
        };
    }

    // Drives to a field point while turning to the target heading. With turnFirst, it turns
    // in place until within TURN_FIRST_TOLERANCE before moving. Returns true once within
    // tolerance of both position and heading.
    private boolean driveToPoint(double goalForward, double goalRight, double goalHeading,
            double forward, double right, double heading, boolean turnFirst) {
        double headingError = normalizeAngle(heading - goalHeading);
        double[] move = translationToward(goalForward, goalRight, forward, right);
        if (move == null && Math.abs(headingError) <= HEADING_TOLERANCE_RADIANS) {
            drive.stop();
            return true;
        }
        if (move == null || (turnFirst && Math.abs(headingError) > TURN_FIRST_TOLERANCE_RADIANS)) {
            move = new double[] {0, 0};
        }
        drive.driveFieldCentric(move[0], move[1],
                holdHeadingTurn(heading, goalHeading, HEADING_KP, MAX_TURN_POWER), heading);
        return false;
    }

    // Turns at a fixed power while holding position; spinning on mecanum wheels otherwise
    // walks the robot sideways (about 11in over a quarter turn on the robot).
    private void holdPositionWhileTurning(double goalForward, double goalRight,
            double clockwise, double forward, double right, double heading) {
        double[] move = translationToward(goalForward, goalRight, forward, right);
        if (move == null) {
            move = new double[] {0, 0};
        }
        drive.driveFieldCentric(move[0], move[1], clockwise, heading);
    }

    // Field-frame drive powers toward the goal, or null if already within tolerance.
    private static double[] translationToward(
            double goalForward, double goalRight, double forward, double right) {
        double errorForward = goalForward - forward;
        double errorRight = goalRight - right;
        double distance = Math.hypot(errorForward, errorRight);
        if (distance <= POSITION_TOLERANCE_INCHES) {
            return null;
        }
        double magnitude = clip(distance * TRANSLATION_KP, MIN_DRIVE_POWER, MAX_DRIVE_POWER);
        return new double[] {errorForward / distance * magnitude, errorRight / distance * magnitude};
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

    private static double clampRange(double value, double low, double high) {
        return low > high ? (low + high) / 2 : clip(value, low, high);
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
