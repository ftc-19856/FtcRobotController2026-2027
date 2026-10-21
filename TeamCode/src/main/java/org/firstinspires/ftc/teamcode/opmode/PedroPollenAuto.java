package org.firstinspires.ftc.teamcode.opmode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import java.util.function.DoubleSupplier;

/**
 * Empty the preloaded pollen into the garden, track visible pollen with the intake running,
 * then follow a Pedro path to park.
 */
@Autonomous(name = "Pedro Pollen Collect And Park", group = "Competition")
public final class PedroPollenAuto extends OpMode {
    public static final double COLLECTION_SECONDS = 20.0;
    public static final double PARK_SECONDS = 10.0;
    public static final int STARTING_POLLEN = 4;
    public static final int POLLEN_CAPACITY = 4;

    // Absolute Pedro field coordinates in inches, with headings in radians.
    public static final double START_X = 132.4;
    public static final double START_Y = 9.8;
    public static final double START_HEADING = Math.toRadians(90);
    public static double PARK_X = START_X;
    public static double PARK_Y = START_Y;
    public static double PARK_HEADING = START_HEADING;

    // Where the robot sits, intake facing the garden, to push its pollen out. NaN means not
    // measured yet: the garden step is skipped and the preload stays in the robot.
    // Measured by pushing the robot from the start pose with the Localization Test, which
    // read (84.028, 0.2271, 0.064 rad) relative to the start: 84in straight ahead.
    public static double GARDEN_X = START_X - 0.2271;
    public static double GARDEN_Y = START_Y + 84.028;
    public static double GARDEN_HEADING = START_HEADING + 0.064;
    private static final double GARDEN_DRIVE_TIMEOUT_SECONDS = 6.0;
    // How far from the garden pose still counts as there; farther and it keeps the pollen.
    private static final double GARDEN_TOLERANCE = 4.0;
    private static final double EJECT_SECONDS = 1.5;

    private static final double MIN_AREA_PERCENT = 0.1;
    public static double CAMERA_HEIGHT = 8.5;
    public static double CAMERA_FORWARD_OFFSET = 7.5;
    public static double CAMERA_RIGHT_OFFSET = 0;
    private static final double POLLEN_CENTER_HEIGHT = 2.85 / 2;
    private static final double MAX_RANGE = 144;
    private static final double FIELD_SIZE = 144;
    // A square robot sweeps a larger footprint when turning than when axis-aligned.
    static final double TURN_CLEARANCE = Math.hypot(RobotConfig.ROBOT_LENGTH_INCHES / 2,
            RobotConfig.ROBOT_WIDTH_INCHES / 2) + 1;
    private static final double SEARCH_RADIANS_PER_SECOND = Math.toRadians(70);
    private static final double ACQUIRE_SETTLE_SECONDS = 0.12;
    private static final double ACQUIRE_TIMEOUT_SECONDS = 0.7;
    private static final double ALIGN_TIMEOUT_SECONDS = 2.0;
    private static final double APPROACH_TIMEOUT_SECONDS = 5.0;
    private static final double FINISH_PICKUP_SECONDS = 0.35;
    private static final double POST_PICKUP_BRAKE_SECONDS = 0.25;
    // Intake mouth is assumed at the front of the 18-inch robot. Travel three
    // inches past first contact to pull the pollen in before starting another scan.
    public static double INTAKE_FORWARD_OFFSET = RobotConfig.ROBOT_LENGTH_INCHES / 2;
    public static double PICKUP_OVERTRAVEL = 3.0;
    private static final double POSITION_TOLERANCE = 1.5;
    private static final double HEADING_TOLERANCE = Math.toRadians(5);

    enum State { GARDEN, TRACK, FULL, PARK, DONE }

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private Follower follower;
    private Limelight3A limelight;
    private DcMotorEx intake;
    private State state = State.DONE;
    private Pose parkPose;
    private enum IntakePhase { CLEARANCE, SEARCH, ACQUIRE, ALIGN, APPROACH, FINISH, BRAKE }
    private IntakePhase intakePhase;
    private Pose scanAnchor;
    private Pose approachGoal;
    private double scanHeading;
    private double lastScanTime;
    private final TargetSamples samples = new TargetSamples();
    private Pose activeBall;
    private Pose lastPickupBall;
    private double lastPickupTime = Double.NEGATIVE_INFINITY;
    private Command routine;
    // Default match preload is four. Intake tests can set the actual starting
    // load during INIT; after START only pickup attempts change the estimate.
    private int heldPollen = STARTING_POLLEN;
    private boolean previousLoadUp;
    private boolean previousLoadDown;
    static boolean hasCapacity(int held) {
        return held >= 0 && held < POLLEN_CAPACITY;
    }

    static int estimateAfterPickup(int held) {
        return hasCapacity(held) ? held + 1 : held;
    }

    // Injectable clock/callbacks let tests verify the real Ivy command lifecycle.
    static Command collectCommand(DoubleSupplier elapsed, Runnable start,
            Runnable track, Runnable cleanup) {
        return Command.build()
                .setStart(start)
                .setExecute(() -> {
                    if (elapsed.getAsDouble() < COLLECTION_SECONDS) track.run();
                })
                .setDone(() -> elapsed.getAsDouble() >= COLLECTION_SECONDS)
                .setEnd(reason -> cleanup.run());
    }

    static Command routineCommand(DoubleSupplier elapsed, Command collect,
            Command park, Runnable finish) {
        return sequential(collect,
                park.until(() -> elapsed.getAsDouble() >= COLLECTION_SECONDS + PARK_SECONDS),
                instant(finish));
    }

    @Override
    public void init() {
        Scheduler.reset();
        // Pedro uses this setting when its drivetrain is stopped. Path-following
        // powers and Foresight tuning remain unchanged.
        Constants.drivetrainConfig.manualBrakeMode.set(true);
        follower = Constants.create(hardwareMap);
        follower.holdEnd.set(true);
        follower.setPose(new Pose(START_X, START_Y, START_HEADING));
        intake = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR);
        intake.setPower(0);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        heldPollen = STARTING_POLLEN;
        previousLoadUp = false;
        previousLoadDown = false;
        limelight = hardwareMap.get(Limelight3A.class, RobotConfig.LIMELIGHT);
        limelight.pipelineSwitch(RobotConfig.POLLEN_PIPELINE_INDEX);
        limelight.start();
        telemetry.addLine("Intake test: set actual starting load with D-pad UP/DOWN before START.");
        telemetry.addData("Starting pollen", "%d / %d", heldPollen, POLLEN_CAPACITY);
        telemetry.addData("Start pose (in, deg)", "%.1f, %.1f, %.1f",
                START_X, START_Y, Math.toDegrees(START_HEADING));
        telemetry.addData("Robot size (in)", "%.0f x %.0f",
                RobotConfig.ROBOT_LENGTH_INCHES, RobotConfig.ROBOT_WIDTH_INCHES);
        telemetry.addLine("Keep still while Pinpoint calibrates.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Rising-edge controls prevent holding a button from changing the count repeatedly.
        if (gamepad1.dpad_up && !previousLoadUp && !gamepad1.dpad_down) {
            heldPollen = Math.min(POLLEN_CAPACITY, heldPollen + 1);
        }
        if (gamepad1.dpad_down && !previousLoadDown && !gamepad1.dpad_up) {
            heldPollen = Math.max(0, heldPollen - 1);
        }
        previousLoadUp = gamepad1.dpad_up;
        previousLoadDown = gamepad1.dpad_down;
        follower.update();
        telemetry.addLine("D-pad UP/DOWN: set actual starting pollen; empty robot = 0.");
        telemetry.addData("Starting pollen", "%d / %d", heldPollen, POLLEN_CAPACITY);
        telemetry.addData("Start pose (in, deg)", "%.1f, %.1f, %.1f",
                START_X, START_Y, Math.toDegrees(START_HEADING));
        telemetry.addLine(gardenMeasured()
                ? "Garden first, then collect"
                : "Garden position not set: skipping the garden");
        telemetry.addLine(hasCapacity(heldPollen) || gardenMeasured()
                ? "Ready to intake" : "Full: intake disabled");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(new Pose(START_X, START_Y, START_HEADING));
        parkPose = new Pose(PARK_X, PARK_Y, PARK_HEADING);
        autoTimer.reset();
        lastPickupBall = null;
        lastPickupTime = Double.NEGATIVE_INFINITY;
        routine = routineCommand(autoTimer::seconds, sequential(
                lazy(this::gardenCommand)
                        .until(() -> autoTimer.seconds() >= COLLECTION_SECONDS),
                collectCommand(autoTimer::seconds, () -> {
                    state = hasCapacity(heldPollen) ? State.TRACK : State.FULL;
                    intake.setPower(hasCapacity(heldPollen) ? 1 : 0);
                    if (hasCapacity(heldPollen)) beginSearch();
                }, this::trackPollen, () -> {
                    stopIntake();
                    stopDrive();
                })),
                lazy(this::parkCommand), this::finish);
        Scheduler.schedule(routine);
    }

    @Override
    public void loop() {
        Pose pose = follower.pose();
        // The global cutoff also covers delayed scheduling or a stalled command.
        if (!Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading())
                || autoTimer.seconds() >= COLLECTION_SECONDS + PARK_SECONDS) {
            if (routine != null) Scheduler.cancel(routine);
            finish();
        } else {
            Scheduler.execute();
        }
        follower.update();
        telemetry.addData("State", state);
        telemetry.addData("Seconds", "%.1f", autoTimer.seconds());
        telemetry.addData("Estimated pollen held", "%d / %d", heldPollen, POLLEN_CAPACITY);
        if (state == State.TRACK) {
            telemetry.addData("Intake phase", intakePhase);
            if (approachGoal != null) telemetry.addData("Pickup pose", approachGoal);
        }
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }

    private void trackPollen() {
        if (!hasCapacity(heldPollen)) {
            state = State.FULL;
            intake.setPower(0);
            stopDrive();
            return;
        }
        state = State.TRACK;
        intake.setPower(1);
        Pose current = follower.pose();
        switch (intakePhase) {
            case CLEARANCE:
                if (near(current, scanAnchor, 0.5)) startScan();
                break;
            case SEARCH:
                Pose ball = detectedBall();
                if (ball != null) {
                    samples.reset();
                    // Settle briefly before sampling; do not range-measure while spinning.
                    follower.hold(current);
                    enterPhase(IntakePhase.ACQUIRE);
                } else {
                    double now = phaseTimer.seconds();
                    scanHeading += SEARCH_RADIANS_PER_SECOND * Math.max(0, now - lastScanTime);
                    lastScanTime = now;
                    // Pedro holds the center of rotation while the camera sweeps.
                    follower.hold(new Pose(scanAnchor.x(), scanAnchor.y(), scanHeading));
                }
                break;
            case ACQUIRE:
                if (phaseTimer.seconds() >= ACQUIRE_SETTLE_SECONDS) {
                    LLResult result = limelight.getLatestResult();
                    Pose sample = ballFromResult(result);
                    if (sample != null) {
                        samples.add(sample, result.getTimestamp());
                    }
                    Pose lockedBall = samples.agreed();
                    if (lockedBall != null) {
                        activeBall = lockedBall;
                        approachGoal = pickupPose(current, lockedBall);
                        if (approachGoal == null) {
                            beginSearch();
                        } else {
                            follower.hold(new Pose(current.x(), current.y(), approachGoal.heading()));
                            enterPhase(IntakePhase.ALIGN);
                        }
                        break;
                    }
                }
                if (phaseTimer.seconds() >= ACQUIRE_TIMEOUT_SECONDS) beginSearch();
                break;
            case ALIGN:
                if (Math.abs(angleError(current.heading(), approachGoal.heading()))
                        <= HEADING_TOLERANCE) {
                    // One frozen target and one Pedro path: near-target camera loss
                    // cannot cause stop/start pulses or continuously change the path.
                    if (near(current, approachGoal, POSITION_TOLERANCE)) {
                        beginFinish();
                    } else {
                        follower.follow(line(current, approachGoal).constant(approachGoal.heading()));
                        enterPhase(IntakePhase.APPROACH);
                    }
                } else if (phaseTimer.seconds() >= ALIGN_TIMEOUT_SECONDS) beginSearch();
                break;
            case APPROACH:
                if (near(current, approachGoal, POSITION_TOLERANCE)) {
                    beginFinish();
                } else if (phaseTimer.seconds() >= APPROACH_TIMEOUT_SECONDS) {
                    // An incomplete approach is not counted as a pickup.
                    beginSearch();
                }
                break;
            case FINISH:
                if (phaseTimer.seconds() >= FINISH_PICKUP_SECONDS) {
                    heldPollen = estimateAfterPickup(heldPollen);
                    lastPickupBall = activeBall;
                    lastPickupTime = autoTimer.seconds();
                    stopDrive();
                    if (hasCapacity(heldPollen)) {
                        enterPhase(IntakePhase.BRAKE);
                    } else {
                        state = State.FULL;
                        stopIntake();
                        stopDrive();
                    }
                }
                break;
            case BRAKE:
                if (phaseTimer.seconds() >= POST_PICKUP_BRAKE_SECONDS) beginSearch();
                break;
        }
    }

    private void beginSearch() {
        Pose current = follower.pose();
        scanAnchor = clearancePose(current);
        approachGoal = null;
        activeBall = null;
        samples.reset();
        if (!near(current, scanAnchor, 0.5)) {
            follower.follow(line(current, scanAnchor).constant(current.heading()));
            enterPhase(IntakePhase.CLEARANCE);
        } else {
            startScan();
        }
    }

    private void startScan() {
        scanHeading = follower.pose().heading();
        lastScanTime = 0;
        enterPhase(IntakePhase.SEARCH);
        follower.hold(new Pose(scanAnchor.x(), scanAnchor.y(), scanHeading));
    }

    private void beginFinish() {
        // Zero-power BRAKE stops the wheels instead of commanding small hold
        // corrections while the intake finishes pulling the pollen inside.
        stopDrive();
        enterPhase(IntakePhase.FINISH);
    }

    private void enterPhase(IntakePhase next) {
        intakePhase = next;
        phaseTimer.reset();
    }

    private Pose detectedBall() {
        return ballFromResult(limelight.getLatestResult());
    }

    private Pose ballFromResult(LLResult result) {
        if (result == null || !result.isValid() || result.getStaleness() > 150
                || result.getPipelineIndex() != RobotConfig.POLLEN_PIPELINE_INDEX
                || !Double.isFinite(result.getTa()) || result.getTa() < MIN_AREA_PERCENT) return null;
        Pose ball = ballFieldPose(follower.pose(), result.getTx(), result.getTy());
        // Give the camera time to switch away from the ball just collected;
        // otherwise a lingering detection can immediately repeat the same pickup.
        if (ball != null && lastPickupBall != null
                && autoTimer.seconds() - lastPickupTime < 1.0
                && near(ball, lastPickupBall, 6.0)) return null;
        // Skip wall targets which cannot be reached with clearance for the next turn.
        return ball != null && pickupPose(follower.pose(), ball) != null ? ball : null;
    }

    static boolean gardenMeasured() {
        return Double.isFinite(GARDEN_X) && Double.isFinite(GARDEN_Y)
                && Double.isFinite(GARDEN_HEADING);
    }

    // Drives to the garden and runs the intake backwards to empty the robot, freeing its
    // capacity for collection. Skipped when empty or when the garden hasn't been measured.
    private Command gardenCommand() {
        if (heldPollen <= 0 || !gardenMeasured()) return instant(() -> { });
        state = State.GARDEN;
        stopIntake();
        Pose current = follower.pose();
        Pose garden = new Pose(GARDEN_X, GARDEN_Y, GARDEN_HEADING);
        return sequential(
                race(sequential(
                                follow(follower, line(current, garden).linear(current, garden)),
                                waitUntil(() -> !follower.isBusy())),
                        waitMs(GARDEN_DRIVE_TIMEOUT_SECONDS * 1000)),
                lazy(() -> {
                    // Only empty out at the garden; otherwise the pollen would land on
                    // the field floor, so keep it and skip collecting.
                    if (!near(follower.pose(), garden, GARDEN_TOLERANCE)) {
                        return instant(this::stopDrive);
                    }
                    return sequential(
                            instant(() -> {
                                stopDrive();
                                intake.setPower(-1);
                            }),
                            waitMs(EJECT_SECONDS * 1000),
                            instant(() -> {
                                stopIntake();
                                heldPollen = 0;
                            }));
                }))
                .setEnd(reason -> stopIntake());
    }

    private Command parkCommand() {
        state = State.PARK;
        stopIntake();
        Pose current = follower.pose();
        if (Math.hypot(current.x() - parkPose.x(), current.y() - parkPose.y())
                <= POSITION_TOLERANCE) {
            // Avoid a zero-length path; hold handles a heading-only adjustment.
            return sequential(hold(follower, parkPose), waitUntil(this::parkReached));
        }
        return sequential(
                follow(follower, line(current, parkPose).linear(current, parkPose)),
                // Ivy 1.1.1 follow ends at parametric end. Keep Pedro's endpoint
                // correction active until its built-in completion constraints finish.
                waitUntil(() -> !follower.isBusy()));
    }

    private boolean parkReached() {
        Pose pose = follower.pose();
        return Math.hypot(pose.x() - parkPose.x(), pose.y() - parkPose.y())
                <= POSITION_TOLERANCE
                && Math.abs(angleError(pose.heading(), parkPose.heading())) <= HEADING_TOLERANCE;
    }

    private void finish() {
        state = State.DONE;
        stopIntake();
        stopDrive();
    }

    static Pose ballFieldPose(Pose robot, double tx, double ty) {
        double ahead = cameraRange(ty);
        if (!Double.isFinite(tx) || !Double.isFinite(ahead) || Math.abs(tx) >= 90) return null;
        double forward = CAMERA_FORWARD_OFFSET + ahead;
        double left = -CAMERA_RIGHT_OFFSET - ahead * Math.tan(Math.toRadians(tx));
        double c = Math.cos(robot.heading()), s = Math.sin(robot.heading());
        Pose ball = new Pose(robot.x() + forward * c - left * s,
                robot.y() + forward * s + left * c);
        return ball.x() >= 0 && ball.x() <= FIELD_SIZE
                && ball.y() >= 0 && ball.y() <= FIELD_SIZE ? ball : null;
    }

    static Pose clearancePose(Pose current) {
        return new Pose(clamp(current.x(), TURN_CLEARANCE, FIELD_SIZE - TURN_CLEARANCE),
                clamp(current.y(), TURN_CLEARANCE, FIELD_SIZE - TURN_CLEARANCE), current.heading());
    }

    static Pose pickupPose(Pose robot, Pose ball) {
        double dx = ball.x() - robot.x(), dy = ball.y() - robot.y();
        double distance = Math.hypot(dx, dy);
        if (distance < 1e-6) return null;
        double heading = Math.atan2(dy, dx);
        double travel = Math.max(0, distance - INTAKE_FORWARD_OFFSET + PICKUP_OVERTRAVEL);
        Pose goal = new Pose(robot.x() + dx / distance * travel,
                robot.y() + dy / distance * travel, heading);
        Pose clamped = clearancePose(goal);
        // Do not claim a pickup when clamping leaves the intake short of the ball.
        return near(goal, clamped, 1.0) ? clamped : null;
    }

    static final class TargetSamples {
        private final double[] xs = new double[3];
        private final double[] ys = new double[3];
        private int count;
        private double lastTimestamp = Double.NaN;

        void reset() { count = 0; lastTimestamp = Double.NaN; }

        void add(Pose point, double timestamp) {
            if (!Double.isFinite(timestamp) || timestamp == lastTimestamp) return;
            lastTimestamp = timestamp;
            xs[count % 3] = point.x();
            ys[count % 3] = point.y();
            count++;
        }

        Pose agreed() {
            if (count < 3) return null;
            double[] sortedX = xs.clone(), sortedY = ys.clone();
            java.util.Arrays.sort(sortedX);
            java.util.Arrays.sort(sortedY);
            Pose median = new Pose(sortedX[1], sortedY[1]);
            for (int i = 0; i < 3; i++) {
                if (Math.hypot(xs[i] - median.x(), ys[i] - median.y()) > 4) return null;
            }
            return median;
        }
    }

    private static boolean near(Pose a, Pose b, double tolerance) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y()) <= tolerance;
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    static double cameraRange(double ty) {
        if (!Double.isFinite(ty) || ty >= 0 || ty <= -90) return Double.NaN;
        double range = (CAMERA_HEIGHT - POLLEN_CENTER_HEIGHT) / Math.tan(Math.toRadians(-ty));
        return range > 0 && range <= MAX_RANGE ? range : Double.NaN;
    }

    private static double angleError(double a, double b) {
        return Math.atan2(Math.sin(a - b), Math.cos(a - b));
    }

    private void stopDrive() {
        follower.stop();
        // stop() changes Pedro mode; use its drivetrain stop for immediate zero
        // power without briefly switching the motors into FLOAT.
        follower.drivetrain.stop();
    }

    private void stopIntake() {
        if (intake != null) intake.setPower(0);
    }

    @Override
    public void stop() {
        if (routine != null) Scheduler.cancel(routine);
        Scheduler.reset();
        if (follower != null) stopDrive();
        stopIntake();
        if (limelight != null) limelight.stop();
    }
}
