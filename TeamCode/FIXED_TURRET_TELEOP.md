# Fixed Turret Intake Shooter

Select **Fixed Turret Intake Shooter** under TeleOp. `init()` is completely
empty: no hardware lookup, motor power, servo movement, calibration, or encoder
reset. Hardware setup and reference capture happen only when START is pressed.

- **Left stick:** forward/back and strafe, relative to the robot.
- **Right stick X:** turn (right stick right turns clockwise).
- Driving matches the current `StarterTeleOp` exactly: `forward = -left_stick_y`,
  `right = left_stick_x`, `clockwise = right_stick_x`, sent to
  `MecanumDrive.driveRobotCentric(forward, right, clockwise)`. There is no custom
  slow mode or additional deadband/scaling. The shared drivetrain handles wheel
  mixing, normalization, motor directions and BRAKE behavior.
- Like Starter TeleOp, it constructs `MecanumDrive` and resets IMU yaw. Both
  happen at START here so INIT remains empty. No Pinpoint pose reset is used.

- **X:** toggle intake/transfer motor `intake` between **-1.0** and off.
- **Y:** independently toggle shooter motor `shooter` between **-1.0** and off.
- Both start **off**. Holding a button cannot repeatedly toggle, and buttons
  held across START must be released and pressed again before switching on.
- Turret motor `turret` holds the encoder position captured at START, using
  `RUN_TO_POSITION`, BRAKE and a maximum holding power of **0.2**. No encoder
  reset or turret home movement is performed; its encoder must be connected.
- **Hold A:** move positional servo `blocker` to the **0.2 ready-to-shoot position**.
  Release A to return it to **0.0 blocking**. B has no function. This is a plain
  `if (gamepad1.a) ... else ...` in `loop()`, with no blocker toggle or helper.
- No servo command is sent during INIT or at START. The first active loop
  applies the current button state; without A held, it commands closed. No
  starting-position estimate is needed. PWM is enabled when supported by
  `ServoImplEx`. The physical endpoint calibration still needs verification.

`Servo.getPosition()` is the last commanded value, not measured angle. This
mode does not change servo PWM endpoints, range scaling or direction. Adjust
`BLOCKER_BLOCKED_POSITION` and `BLOCKER_CLEAR_POSITION` if the actual mechanism
requires different endpoints. If direct commands still produce no physical
motion, inspect the configured servo port, connection and power.

This is a driving TeleOp using the same robot-centric drivetrain as Starter TeleOp. Turret
"fixed" means fixed relative to the chassis, not a field-direction lock.
It does not use vision or command the hood. STOP brakes the drivetrain, zeros
all three mechanism motor powers and commands the blocker closed. The -1 intake/shooter
directions match the user's existing `TestIntakeTransferShooter` working copy.
That staged/uncommitted file is preserved separately from this new mode.

Servo API reference: [FTC Servo.getPosition](https://javadoc.io/static/org.firstinspires.ftc/RobotCore/10.2.0/com/qualcomm/robotcore/hardware/Servo.html).
