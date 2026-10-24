package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotConfig;

/** Reports configured device types and encoder readings without moving mechanisms. */
@TeleOp(name = "Shooter Hardware Setup", group = "Test")
public final class ShooterHardwareSetup extends OpMode {
    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(100);
        showHardware();
    }

    @Override
    public void init_loop() { showHardware(); }

    @Override
    public void loop() { showHardware(); }

    private void showHardware() {
        telemetry.addLine("Inspection only: no power, servo position, or encoder reset commands.");
        telemetry.addLine("Configured device type does not verify its physical port or wiring.");
        describe(RobotConfig.INTAKE_MOTOR);
        describe(RobotConfig.SHOOTER_MOTOR);
        describe(RobotConfig.TURRET_MOTOR);
        describe(RobotConfig.HOOD);
        describe(RobotConfig.BLOCKER_SERVO);
        telemetry.addLine("Encoder numbers alone do not establish home, gearing, or travel limits.");
        telemetry.update();
    }

    private void describe(String name) {
        HardwareDevice device = hardwareMap.tryGet(HardwareDevice.class, name);
        if (device == null) {
            telemetry.addData(name, "not present in active robot configuration");
        } else if (device instanceof DcMotor) {
            DcMotor motor = (DcMotor) device;
            telemetry.addData(name + " type", "DC motor");
            telemetry.addData(name + " encoder ticks", motor.getCurrentPosition());
            telemetry.addData(name + " run mode", motor.getMode());
            if (device instanceof DcMotorEx) {
                telemetry.addData(name + " ticks/sec", "%.1f", ((DcMotorEx) device).getVelocity());
            }
        } else if (device instanceof Servo) {
            telemetry.addData(name + " type", "positional servo");
            telemetry.addData(name + " last commanded position", "%.3f", ((Servo) device).getPosition());
            telemetry.addLine(name + ": last command is NOT measured mechanism position");
        } else if (device instanceof CRServo) {
            telemetry.addData(name + " type", "continuous-rotation servo");
        } else {
            telemetry.addData(name + " type", device.getDeviceName());
        }
    }
}
