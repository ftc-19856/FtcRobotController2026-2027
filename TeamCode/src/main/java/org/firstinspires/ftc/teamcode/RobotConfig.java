package org.firstinspires.ftc.teamcode;

/** Hardware names and robot-specific measurements. */
public final class RobotConfig {
    public static final String FRONT_LEFT_MOTOR = "frontLeftMotor";
    public static final String BACK_LEFT_MOTOR = "backLeftMotor";
    public static final String FRONT_RIGHT_MOTOR = "frontRightMotor";
    public static final String BACK_RIGHT_MOTOR = "backRightMotor";
    public static final String INTAKE_MOTOR = "intake";
    public static final String IMU = "imu";
    public static final String PINPOINT = "pinpoint";
    public static final String LIMELIGHT = "limelight";
    public static final int POLLEN_PIPELINE_INDEX = 0;

    // X pod: positive is left of the tracking point.
    public static final double PINPOINT_X_OFFSET_INCHES = -0.752;
    // Y pod: positive is forward of the tracking point.
    public static final double PINPOINT_Y_OFFSET_INCHES = -1.769;

    private RobotConfig() {
    }
}
