package OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Components.Odometry;

// Drives from the start point to a ball at a known offset, runs the intake while
// approaching, then returns to the start point. No turning - the robot strafes
// diagonally to the target with mecanum kinematics, holding heading constant the
// whole time. Position is closed-loop via the calibrated dead wheel odometry, so it
// self-corrects instead of relying on fixed timing/power.
//
// The ball's position (relative to the start point and starting heading, since
// Odometry resets to (0,0) on init) was measured by manually driving the robot there
// with OdometryTest and reading its telemetry.
@Autonomous(name = "IntakeAndReturn", group = "Autonomous")
public class IntakeAndReturn extends OpMode {
    // Ball position relative to the start point, in inches (measured with OdometryTest).
    private static final double BALL_FORWARD = -52.2552;
    private static final double BALL_RIGHT = 54.5694;

    private static final double POSITION_TOLERANCE_IN = 1.5;
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MIN_DRIVE_POWER = 0.15;
    private static final double DRIVE_KP = 0.03;

    private static final double INTAKE_DWELL_SECONDS = 1.0;
    private static final double DRIVE_TIMEOUT_SECONDS = 8.0;

    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;
    DcMotorEx intake;

    Odometry odometry;
    final ElapsedTime stateTimer = new ElapsedTime();

    enum State {
        DRIVE_TO_BALL,
        COLLECT,
        RETURN_TO_START,
        DONE
    }

    State state;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        odometry = new Odometry(hardwareMap);

        telemetry.addData("Status", "Initialized");
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

        double forward = odometry.getForward(DistanceUnit.INCH);
        double right = odometry.getRight(DistanceUnit.INCH);

        switch (state) {
            case DRIVE_TO_BALL:
                intake.setPower(1);
                if (driveToward(BALL_FORWARD, BALL_RIGHT, forward, right)
                        || stateTimer.seconds() > DRIVE_TIMEOUT_SECONDS) {
                    stopDrive();
                    state = State.COLLECT;
                    stateTimer.reset();
                }
                break;

            case COLLECT:
                intake.setPower(1);
                if (stateTimer.seconds() > INTAKE_DWELL_SECONDS) {
                    state = State.RETURN_TO_START;
                    stateTimer.reset();
                }
                break;

            case RETURN_TO_START:
                intake.setPower(0);
                if (driveToward(0, 0, forward, right)
                        || stateTimer.seconds() > DRIVE_TIMEOUT_SECONDS) {
                    stopDrive();
                    state = State.DONE;
                }
                break;

            case DONE:
                stopDrive();
                intake.setPower(0);
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Forward (in)", forward);
        telemetry.addData("Right (in)", right);
        telemetry.update();
    }

    // Drives toward (targetForward, targetRight). Returns true once within tolerance.
    private boolean driveToward(double targetForward, double targetRight, double forward, double right) {
        double errorForward = targetForward - forward;
        double errorRight = targetRight - right;

        if (Math.hypot(errorForward, errorRight) < POSITION_TOLERANCE_IN) {
            return true;
        }

        double powerForward = powerFor(errorForward);
        double powerRight = powerFor(errorRight);

        double frontLeftPower = powerForward + powerRight;
        double backLeftPower = powerForward - powerRight;
        double frontRightPower = powerForward - powerRight;
        double backRightPower = powerForward + powerRight;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower), Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

        frontLeftMotor.setPower(frontLeftPower / max);
        backLeftMotor.setPower(backLeftPower / max);
        frontRightMotor.setPower(frontRightPower / max);
        backRightMotor.setPower(backRightPower / max);

        return false;
    }

    private double powerFor(double error) {
        double power = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, error * DRIVE_KP));
        if (Math.abs(power) > 0 && Math.abs(power) < MIN_DRIVE_POWER) {
            power = Math.copySign(MIN_DRIVE_POWER, power);
        }
        return power;
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
