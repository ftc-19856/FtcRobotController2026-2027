package org.firstinspires.ftc.teamcode.opmode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Shoots the preloaded pollen from the starting spot, then drives straight forward out of
 * the starting area for leave points and parks there. The shooter is fixed, so place the
 * robot aimed at the goal.
 */
@Autonomous(name = "Pedro Shoot And Leave", group = "Competition")
public final class PedroShootAndLeave extends OpMode {
    // Same start pose as Pedro Pollen Collect And Park.
    private static final Pose START = new Pose(PedroPollenAuto.START_X,
            PedroPollenAuto.START_Y, PedroPollenAuto.START_HEADING);

    // Untested guesses until tuned on the robot.
    private static final double SHOOTER_SPIN_UP_SECONDS = 1.5;
    // The intake drives the transfer, pushing pollen past the open blocker into the
    // flywheel; run it long enough to feed all four.
    private static final double FEED_SECONDS = 3.0;
    // How far to drive straight ahead after shooting. Set it to clear the starting area.
    public static double LEAVE_FORWARD_INCHES = 24.0;
    private static final double LEAVE_TIMEOUT_SECONDS = 5.0;

    private enum State { SPIN_UP, FEED, LEAVE, DONE }

    private final ElapsedTime stateTimer = new ElapsedTime();
    private Follower follower;
    private DcMotorEx intake;
    private AutoShooter shooter;
    private State state = State.DONE;

    @Override
    public void init() {
        Constants.drivetrainConfig.manualBrakeMode.set(true);
        follower = Constants.create(hardwareMap);
        follower.holdEnd.set(true);
        follower.setPose(START);
        intake = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR);
        // Matches Pedro Pollen Collect And Park: positive power pulls pollen in.
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0);
        shooter = new AutoShooter(hardwareMap);
        telemetry.addLine("Aim the robot at the goal. Keep still while Pinpoint calibrates.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
    }

    @Override
    public void start() {
        follower.setPose(START);
        shooter.start();
        shooter.spinUp();
        enter(State.SPIN_UP);
    }

    @Override
    public void loop() {
        switch (state) {
            case SPIN_UP:
                if (stateTimer.seconds() >= SHOOTER_SPIN_UP_SECONDS) {
                    shooter.openBlocker();
                    intake.setPower(1);
                    enter(State.FEED);
                }
                break;
            case FEED:
                if (stateTimer.seconds() >= FEED_SECONDS) {
                    intake.setPower(0);
                    shooter.finishShooting();
                    Pose current = follower.pose();
                    Pose leave = new Pose(
                            current.x() + LEAVE_FORWARD_INCHES * Math.cos(current.heading()),
                            current.y() + LEAVE_FORWARD_INCHES * Math.sin(current.heading()),
                            current.heading());
                    follower.follow(line(current, leave).constant(current.heading()));
                    enter(State.LEAVE);
                }
                break;
            case LEAVE:
                if (!follower.isBusy() || stateTimer.seconds() >= LEAVE_TIMEOUT_SECONDS) {
                    stopDrive();
                    enter(State.DONE);
                }
                break;
            case DONE:
                break;
        }
        follower.update();
        telemetry.addData("State", state);
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }

    private void enter(State next) {
        state = next;
        stateTimer.reset();
    }

    private void stopDrive() {
        follower.stop();
        follower.drivetrain.stop();
    }

    @Override
    public void stop() {
        if (follower != null) stopDrive();
        if (intake != null) intake.setPower(0);
        if (shooter != null) shooter.stop();
    }
}
