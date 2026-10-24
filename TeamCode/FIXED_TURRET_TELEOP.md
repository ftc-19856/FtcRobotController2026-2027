# Fixed Turret Intake Shooter

Select **Fixed Turret Intake Shooter** under TeleOp. `init()` is completely
empty: no hardware lookup, motor power, servo movement, calibration, or encoder
reset. Hardware setup and reference capture happen only when START is pressed.

- **Left stick:** forward/back and strafe, relative to the robot.
- **Right stick X:** turn (right stick right turns clockwise).
- **Left bumper:** 35% slow driving for precise positioning.
- Driving uses Pedro's `Mecanum` with `Constants.drivetrainConfig`, preserving
  the exact motor names and direction settings used by autonomous. There is no
  duplicate wheel-direction configuration, IMU calibration, or pose reset.
- Stick inputs have a 0.05 deadband, are normalized for combined driving, then
  scaled for slow mode. Mechanism powers do not change in slow mode.

- Intake/transfer motor `intake` runs continuously at **-1.0** after START.
- Shooter motor `shooter` runs continuously at **-1.0** after START.
- Turret motor `turret` holds the encoder position captured at START, using
  `RUN_TO_POSITION`, BRAKE and a maximum holding power of **0.2**. No encoder
  reset or turret home movement is performed; its encoder must be connected.
- Positional servo `blocker` keeps its existing command at START. **Gamepad 1 A**
  toggles between that starting command and an offset command. Holding A cannot
  repeatedly toggle it. The starting command is not assumed to mean open or closed.
- The default offset is **-0.2** from the captured starting command. Left is the physical
  blocking side and right is clearance; logical servo direction depends on its
  installation. **B** reverses its
  direction while at the starting position. Return with A before reversing if
  already offset. B does not command any servo movement by itself.

An out-of-range offset is rejected rather than clipped. For example, a starting
command of 0.1 cannot move to -0.1; press B to use 0.3 instead. If the SDK reports
no known starting command, toggling is disabled and telemetry explains why.
There is no guessed fallback or automatic movement to a reference position.

`Servo.getPosition()` is the **last commanded logical position**, not measured
physical angle. It cannot detect hand-positioning, slipping, or a power-up angle.
The servo is reported as a 300-degree model, but the requested move is now a
normalized **0.2**, rather than a guaranteed angle. Actual motion depends on
the configured PWM endpoints, range scaling, and mechanism. This mode preserves
the existing servo direction and PWM/range settings.

This is a driving TeleOp with robot-centric Pedro mecanum control. Turret
"fixed" means fixed relative to the chassis, not a field-direction lock.
It does not use vision or command the hood. STOP brakes the drivetrain, zeros
all three mechanism motor powers and leaves the blocker at its last selected
command. The -1 intake/shooter
directions match the user's existing `TestIntakeTransferShooter` working copy.
That staged/uncommitted file is preserved separately from this new mode.

Servo API reference: [FTC Servo.getPosition](https://javadoc.io/static/org.firstinspires.ftc/RobotCore/10.2.0/com/qualcomm/robotcore/hardware/Servo.html).
