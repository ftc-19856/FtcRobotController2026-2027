package Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import Util.Vector2;

public class Drive {
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    public boolean useFieldDirections = true;

    private IMU imu;

    public Drive(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, IMU imu) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.imu = imu;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double getCurrentRotation() {
        return imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
    }

    public void stopMovement() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void zeroRotation() {
        imu.resetYaw();
    }

    private static double[] calculateMotorPowers(double x, double y, double rotation) {
        double frontLeftPower = y + x + rotation;
        double backLeftPower = y - x - rotation;
        double frontRightPower = y - x + rotation;
        double backRightPower = y + x - rotation;

        double maxPower = Math.max(1.0,
                Math.max(Math.abs(frontLeftPower),
                        Math.max(Math.abs(backLeftPower),
                                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

        frontLeftPower /= maxPower;
        backLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backRightPower /= maxPower;

        return new double[]{frontLeftPower, backLeftPower, frontRightPower, backRightPower};
    }

    // frontLeftPower, backLeftPower, frontRightPower, backRightMotor
    public static double[] directionToMotorPower(Vector2 direction, float rotation) {
        return calculateMotorPowers(direction.x, direction.y, rotation);
    }

    // speed is from 0 to 1
    public void moveInDirection(Vector2 direction, float rotation, float speed, Telemetry telemetry) {
        double fieldRotation = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);

        double inputX = direction.x;
        double inputY = direction.y;
        if (useFieldDirections) {
            double cos = Math.cos(fieldRotation);
            double sin = Math.sin(fieldRotation);
            double rotatedX = inputX * cos + inputY * sin;
            double rotatedY = -inputX * sin + inputY * cos;
            inputX = rotatedX;
            inputY = rotatedY;
        }

        double[] powers = calculateMotorPowers(inputX * speed, inputY * speed, rotation * speed);

        double maxReportedPower = Math.max(Math.abs(powers[0]),
                Math.max(Math.abs(powers[1]),
                        Math.max(Math.abs(powers[2]), Math.abs(powers[3]))));

        telemetry.addData("Front Right Pwr", powers[2]);
        telemetry.addData("Front Left Pwr", powers[0]);
        telemetry.addData("Back Right Pwr", powers[3]);
        telemetry.addData("Back Left Pwr", powers[1]);
        telemetry.addData("Max pwr value", maxReportedPower);
        telemetry.addData("Dir x", direction.x);
        telemetry.addData("Dir y", direction.y);
        telemetry.addData("Rotation",rotation);
        telemetry.addData("inX",inputX);
        telemetry.addData("inY",inputY);
        // Set motor powers
        frontLeftMotor.setPower(powers[0]);
        backLeftMotor.setPower(powers[1]);
        frontRightMotor.setPower(powers[2]);
        backRightMotor.setPower(powers[3]);
    }
}
