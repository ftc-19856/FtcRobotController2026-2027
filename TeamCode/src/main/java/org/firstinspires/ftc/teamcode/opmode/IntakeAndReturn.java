package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.localization.PinpointOdometry;

@Autonomous(name = "Intake And Return", group = "Competition")
public final class IntakeAndReturn extends OpMode {
    // Pinpoint field coordinates measured from the initialized starting pose.
    private static final double BALL_FORWARD_INCHES = -52.2552;
    private static final double BALL_RIGHT_INCHES = 54.5694;
    private static final double POSITION_TOLERANCE_INCHES = 1.5;
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MIN_DRIVE_POWER = 0.15;
    private static final double TRANSLATION_KP = 0.03;
    private static final double HEADING_KP = 1.5;
    private static final double MAX_TURN_POWER = 0.25;
    private static final double DRIVE_TIMEOUT_SECONDS = 8.0;
    private static final double INTAKE_DWELL_SECONDS = 1.0;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private MecanumDrive drive;
    private PinpointOdometry odometry;
    private DcMotorEx intake;
    private State state;

    private enum State {
        DRIVE_TO_BALL,
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
        telemetry.addLine("Initialized; keep still while the Pinpoint IMU calibrates");
        telemetry.update();
    }

    @Override
    public void start() {
        state = State.DRIVE_TO_BALL;
        stateTimer.reset();
    }

    @Override
    public void loop() {
        odometry.update();
        double forward = odometry.getForwardInches();
        double right = odometry.getRightInches();

        switch (state) {
            case DRIVE_TO_BALL:
                intake.setPower(1);
                if (driveTo(BALL_FORWARD_INCHES, BALL_RIGHT_INCHES, forward, right)
                        || timedOut()) {
                    enter(State.COLLECT);
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
                if (driveTo(0, 0, forward, right) || timedOut()) {
                    enter(State.DONE);
                }
                break;
            case DONE:
                drive.stop();
                intake.setPower(0);
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Forward X (in)", "%.2f", forward);
        telemetry.addData("Right Y (in)", "%.2f", right);
        telemetry.addData("Heading (deg)", "%.1f",
                Math.toDegrees(odometry.getHeadingRadians()));
        telemetry.update();
    }

    private boolean driveTo(
            double targetForward, double targetRight, double forward, double right) {
        double errorForward = targetForward - forward;
        double errorRight = targetRight - right;
        double distance = Math.hypot(errorForward, errorRight);
        if (distance <= POSITION_TOLERANCE_INCHES) {
            drive.stop();
            return true;
        }

        double magnitude = clip(distance * TRANSLATION_KP,
                MIN_DRIVE_POWER, MAX_DRIVE_POWER);
        double fieldForward = errorForward / distance * magnitude;
        double fieldRight = errorRight / distance * magnitude;
        double heading = odometry.getHeadingRadians();
        double clockwise = headingCorrection(heading);
        drive.driveFieldCentric(fieldForward, fieldRight, clockwise, heading);
        return false;
    }

    private boolean timedOut() {
        return stateTimer.seconds() >= DRIVE_TIMEOUT_SECONDS;
    }

    static double headingCorrection(double headingRadians) {
        return clip(-headingRadians * HEADING_KP,
                -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    private void enter(State nextState) {
        drive.stop();
        state = nextState;
        stateTimer.reset();
    }

    private static double clip(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    @Override
    public void stop() {
        drive.stop();
        intake.setPower(0);
    }
}
