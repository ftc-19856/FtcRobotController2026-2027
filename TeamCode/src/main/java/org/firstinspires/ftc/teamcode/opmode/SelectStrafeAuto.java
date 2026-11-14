package org.firstinspires.ftc.teamcode.opmode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Select Left or Right Strafe", group = "Competition")
public final class SelectStrafeAuto extends OpMode {
    private static final double STRAFE_INCHES = 12;
    private static final double TIMEOUT_SECONDS = 5;

    private enum Direction {
        LEFT(1), RIGHT(-1);

        final int sign;

        Direction(int sign) { this.sign = sign; }
    }

    private final ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private Direction selected;
    private boolean driving;
    private String status;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.holdEnd.set(true);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        selected = null;
        driving = false;
        status = "Press D-pad Left or Right before START.";
        showTelemetry();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left != gamepad1.dpad_right) {
            selected = gamepad1.dpad_left ? Direction.LEFT : Direction.RIGHT;
        }
        follower.update();
        showTelemetry();
    }

    @Override
    public void start() {
        follower.update();
        Pose start = follower.pose();
        if (selected == null) {
            status = "No direction selected.";
        } else if (!poseReady(start)) {
            status = "Pinpoint pose unavailable.";
        } else {
            double left = selected.sign * STRAFE_INCHES;
            double heading = start.heading();
            Pose end = new Pose(start.x() - left * Math.sin(heading),
                    start.y() + left * Math.cos(heading), heading);
            follower.follow(line(start, end).constant(heading));
            timer.reset();
            driving = true;
            status = "Strafing " + selected;
        }
    }

    @Override
    public void loop() {
        if (driving) {
            follower.update();
            if (!poseReady(follower.pose())) finish("Pinpoint pose unavailable.");
            else if (!follower.isBusy()) finish("Done.");
            else if (timer.seconds() >= TIMEOUT_SECONDS) finish("Strafe timed out.");
        }
        showTelemetry();
    }

    private boolean poseReady(Pose pose) {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private void showTelemetry() {
        telemetry.addData("Auto", status);
        telemetry.addData("Direction", selected == null ? "None" : selected.name());
        telemetry.addData("Distance (in)", STRAFE_INCHES);
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }

    private void finish(String reason) {
        driving = false;
        status = reason;
        follower.stop();
        follower.drivetrain.stop();
    }

    @Override
    public void stop() {
        if (follower != null) finish("Stopped.");
    }
}
