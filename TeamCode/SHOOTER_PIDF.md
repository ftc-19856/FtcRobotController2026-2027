# Shooter velocity PIDF

Fixed Turret Intake Shooter and the shooting/collecting autos now use the Hub's
built-in encoder velocity PIDF through `DcMotorEx`, `RUN_USING_ENCODER`, and
`setVelocity`. The encoder must be connected. Y still toggles the shooter in
TeleOp; autonomous spin-up/feed timing and the blocker controls are unchanged.
INIT commands zero power, and STOP lets the flywheel coast with FLOAT.

The initial speed is **100% of the configured motor's nominal maximum**, in the
same negative direction as the working `setPower(-1)` code. Target ticks/second
are calculated as configured ticks/revolution times configured maximum RPM / 60,
bounded by the Hub's velocity command limit. The motor type in the robot
configuration must match the actual motor. This is not a measured free speed;
load and battery voltage may keep the flywheel below that target. At full
output PID cannot add further power to compensate for a shot.

## Panels

1. INIT a supported OpMode. On the first INIT after app startup, `ShooterVelocity`
   copies the Hub's existing P/I/D/F coefficients rather than inventing tuned gains.
2. Open http://192.168.43.1:8001 and add Configurables and Telemetry widgets.
3. Under `ShooterVelocity`, adjust `P`, `I`, `D`, `F`, and `SPEED_FRACTION` (0 to 1).
   The default fraction is 1; reducing it sets a lower regulated speed.
4. START, then toggle Y on in TeleOp. Observe **Shooter target / actual (ticks/s)**
   and **Shooter error (ticks/s)**. Changes apply during active loops. Gains are
   only resent to the Hub when changed. Invalid settings command zero power.

Panels edits persist between OpModes in the same app session, but are not saved
into Java source. The current Hub coefficients are starting values, not gains
tuned for this flywheel. Physical speed regulation and shooting consistency
still need verification on the robot. The hood and turret are independent of
this velocity controller. TestITS remains the simple open-loop hardware test.

API: [FTC DcMotorEx velocity/PIDF](https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html).
