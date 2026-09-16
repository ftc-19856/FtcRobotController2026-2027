# Starter Bot setup

This implementation is based directly on FTC SDK v12. Configure these hardware
names on the Driver Station:

| Name | Device |
| --- | --- |
| `frontLeftMotor` | Front-left drive motor |
| `backLeftMotor` | Back-left drive motor |
| `frontRightMotor` | Front-right drive motor |
| `backRightMotor` | Back-right drive motor |
| `imu` | Control Hub IMU |
| `pinpoint` | goBILDA Pinpoint |
| `intake` | Intake motor |

The shared drivetrain reverses both left motors and runs all four drive motors
without encoders, with brake behavior at zero power. Verify wheel directions on
blocks before driving. It assumes the Control Hub logo faces up and its USB ports
face forward; update the `RevHubOrientationOnRobot` values in `MecanumDrive` if
the hub is mounted differently.

`Starter TeleOp` is field-centric. It resets Control Hub IMU yaw when the OpMode
starts. The left stick translates, the right stick X axis turns, and A toggles
the optional intake motor.

The Pinpoint uses a forward-tracking pod on X and a strafe-tracking pod on Y.
This code preserves the starter robot's measured convention: forward motion must
increase X, and right motion must increase Y. If either is reversed, change the
corresponding encoder direction in `PinpointOdometry` before autonomous testing.

Pod offsets in `RobotConfig` are measurements from the starter branch. Run
`Pod Offset Calibration` after any mechanical change. Rotate the robot by hand
with the pods touching the floor, then copy stable candidate values into
`RobotConfig`. Confirm them with `Odometry Test`; if rotation-induced drift
doubles instead of shrinking, reverse the sign of that offset.

`Intake And Return` retains the measured starter-branch target of X = -52.2552
inches and right Y = +54.5694 inches. Confirm the direction and target at low
speed after validating both axes with `Odometry Test`.
