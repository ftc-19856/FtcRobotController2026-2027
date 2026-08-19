package OpModes.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import Components.Drive;
import Util.Vector2;

@TeleOp(name="StarterTeleOp", group="TeleOp")
public class StarterTeleOp extends OpMode {
    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;
    IMU imu;


    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");


        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        backRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");
        // Get joystick inputs
        double y = -gamepad1.left_stick_y;   // Forward/backward
        double x = gamepad1.left_stick_x;    // Strafe
        double rx = gamepad1.right_stick_x;  // Rotation

        // Mecanum drive calculations
        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        // Normalize powers so the highest magnitude is 1.0
        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(frontLeftPower),
                        Math.max(
                                Math.abs(backLeftPower),
                                Math.max(
                                        Math.abs(frontRightPower),
                                        Math.abs(backRightPower)
                                )
                        )
                )
        );

        frontLeftPower /= max;
        backLeftPower /= max;
        frontRightPower /= max;
        backRightPower /= max;

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        // Telemetry
        telemetry.addData("FL Power", frontLeftPower);
        telemetry.addData("BL Power", backLeftPower);
        telemetry.addData("FR Power", frontRightPower);
        telemetry.addData("BR Power", backRightPower);
        telemetry.update();


    }
}