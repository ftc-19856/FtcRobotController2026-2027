package org.firstinspires.ftc.teamcode.opmode;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.pedro.AllianceAutonomousPaths;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Shoots the preload from the alliance start pose, then drives the alliance route: path 1
 * off the start (leave points) and path 2 into the corner (park points). The shooter is
 * fixed and the robot starts square on its start pose, which is where it aims from.
 */
public abstract class ShootLeaveParkAuto extends OpMode {
    // Untested guesses until tuned on the robot.
    private static final double SHOOTER_SPIN_UP_MS = 1500;
    // The intake drives the transfer, pushing pollen past the open blocker into the
    // flywheel; run it long enough to feed all four.
    private static final double FEED_MS = 3000;
    private static final double AUTO_LIMIT_SECONDS = 30;

    private final ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private AllianceAutonomousPaths route;
    private DcMotor intake;
    private AutoShooter shooter;
    private Command routine;
    private String phase = "INIT";

    protected abstract AllianceAutonomousPaths createPaths();

    @Override
    public final void init() {
        Scheduler.reset();
        route = createPaths();
        follower = Constants.create(hardwareMap);
        follower.holdEnd.set(true);
        follower.setPose(route.start);
        intake = hardwareMap.get(DcMotor.class, RobotConfig.INTAKE_MOTOR);
        // Matches Pedro Pollen Collect And Park: positive power pulls pollen in.
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0);
        shooter = new AutoShooter(hardwareMap);
        showTelemetry();
    }

    @Override
    public final void init_loop() {
        follower.update();
        showTelemetry();
    }

    @Override
    public final void start() {
        follower.setPose(route.start);
        timer.reset();
        shooter.start();
        routine = sequential(
                sequential(instant(() -> {
                            phase = "SHOOTING";
                            shooter.spinUp();
                        }),
                        waitMs(SHOOTER_SPIN_UP_MS),
                        instant(() -> {
                            shooter.openBlocker();
                            intake.setPower(1);
                        }),
                        waitMs(FEED_MS))
                        .setEnd(reason -> {
                            intake.setPower(0);
                            shooter.finishShooting();
                        }),
                sequential(instant(() -> phase = "LEAVING"),
                        follow(follower, route.path1()),
                        // Ivy 1.1.1 follow ends at parametric end. Let Pedro finish
                        // endpoint correction before starting the parking curve.
                        waitUntil(() -> !follower.isBusy())),
                sequential(instant(() -> phase = "PARKING"),
                        follow(follower, route.path2()),
                        waitUntil(() -> !follower.isBusy())),
                instant(() -> finish("DONE")));
        Scheduler.schedule(routine);
    }

    @Override
    public final void loop() {
        Pose pose = follower.pose();
        if (!Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading())) {
            cancelAndFinish("INVALID POSE");
        } else if (timer.seconds() >= AUTO_LIMIT_SECONDS) {
            cancelAndFinish("30 SECOND STOP");
        } else {
            Scheduler.execute();
        }
        follower.update();
        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addData("Phase", phase);
        telemetry.addData("Start pose", route.start);
        telemetry.addData("Park pose", route.park);
        telemetry.addData("Current pose", follower.pose());
        telemetry.addLine("Place the robot square on the start pose: it shoots from there.");
        telemetry.update();
    }

    private void cancelAndFinish(String reason) {
        if (routine != null) Scheduler.cancel(routine);
        finish(reason);
    }

    private void finish(String reason) {
        phase = reason;
        intake.setPower(0);
        follower.stop();
        follower.drivetrain.stop();
    }

    @Override
    public final void stop() {
        if (routine != null) Scheduler.cancel(routine);
        Scheduler.reset();
        if (follower != null) finish("STOPPED");
        if (shooter != null) shooter.stop();
    }
}
