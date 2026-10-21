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
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.AllianceAutonomousPaths;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Drive to shooting position, pause, then park. No vision or mechanism commands. */
public abstract class AlliancePathAuto extends OpMode {
    public static final double SHOOT_WAIT_MS = 3000;
    private static final double AUTO_LIMIT_SECONDS = 30;
    private final ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private AllianceAutonomousPaths route;
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
        showTelemetry();
    }

    @Override
    public final void init_loop() {
        follower.update();
        showTelemetry();
    }

    static Command routineCommand(Command driveToShoot, Command shootingWait,
            Command driveToPark, Runnable done) {
        return sequential(driveToShoot, shootingWait, driveToPark, instant(done));
    }

    @Override
    public final void start() {
        follower.setPose(route.start);
        timer.reset();
        routine = routineCommand(
                sequential(instant(() -> phase = "PATH 1"),
                        follow(follower, route.path1()),
                        // Ivy 1.1.1 follow ends at parametric end. Let Pedro finish
                        // endpoint correction before starting the 3000 ms pause.
                        waitUntil(() -> !follower.isBusy())),
                sequential(instant(() -> phase = "SHOOTING WAIT (NO MECHANISM CONTROL)"),
                        waitMs(SHOOT_WAIT_MS)),
                sequential(instant(() -> phase = "PARKING"),
                        follow(follower, route.path2()),
                        waitUntil(() -> !follower.isBusy())),
                () -> finish("DONE"));
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
        telemetry.addData("Shooting pose", route.shooting);
        telemetry.addData("Park pose", route.park);
        telemetry.addData("Shooting wait (ms)", SHOOT_WAIT_MS);
        telemetry.addData("Current pose", follower.pose());
        telemetry.addLine("Path/wait test: no vision, intake, shooter, turret, hood or blocker control.");
        telemetry.update();
    }

    private void cancelAndFinish(String reason) {
        if (routine != null) Scheduler.cancel(routine);
        finish(reason);
    }

    private void finish(String reason) {
        phase = reason;
        follower.stop();
        follower.drivetrain.stop();
    }

    @Override
    public final void stop() {
        if (routine != null) Scheduler.cancel(routine);
        Scheduler.reset();
        if (follower != null) finish("STOPPED");
    }
}
