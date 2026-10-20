# Hive vision and shooter setup

The intended scoring target is the hive that is **down**. The existing Pedro
intake/park autonomous is preserved while the new mechanisms are calibrated.

## Confirmed hardware

- `intake`: motor, also mechanically drives the geared transfer.
- `shooter`: shooter motor.
- `turret`: motor; gearing, limits, homing and encoder calibration are unknown.
- `hood`: may be a DC motor rather than a servo. Physical hub port, configured
  type, encoder connection, home and travel limits need confirmation. The earlier
  9/10-of-300-degree gearing description is not used as a servo conversion.
  `RobotConfig.HOOD` records only the name, not an assumed device type.
- `blocker`: positional servo (`RobotConfig.BLOCKER_SERVO`). The reported open
  and closed angles are approximately 0 and 90 degrees, but unconfirmed. Servo
  model, PWM/usable travel, linkage direction and calibrated normalized positions
  are still needed; no open/close position has been commanded.
- No sensor counts pollen. Four preload; maximum held is four. Counts after
  pickup or release must be treated as estimates.

Because the transfer is geared to the intake, running intake also runs transfer.
We need to establish whether the blocker must remain closed during collection
and whether opening it with intake running feeds the shooter.

## Camera visibility and pipelines

The last measured camera pose is level, 8.5 inches above the floor, 7.5 inches
forward of robot center, centered horizontally. The camera is **fixed to the
chassis**. Visibility of floor pollen and hive tags from a shooting position is
not yet established. Hive pipeline numbers are **not assigned**.

Because the camera does not rotate with the turret, camera tx does not shrink
as the turret turns. Turret aiming needs calibrated turret encoder angle/home
and the tag bearing relative to the chassis, accounting for camera and turret
offsets. A PID loop commanding turret power from camera tx alone is unsuitable
for this mounting arrangement. The turret encoder can be calibrated from tick
change over a measured angular movement without knowing the gear tooth counts.

Switching pipelines changes processing, not the physical field of view. One
camera can switch between pollen and hive tags if both fall within its view at
their respective collection/shooting positions. If not, a different shooting
position, changed mounting angle, adjustable mount, or separate camera is needed.
Changing camera pitch also requires updating the intake floor-range calculation.

An AprilTag pipeline normally detects tag IDs; a pipeline index is not a tag ID
or proof that a hive is down. The pipeline/tag mapping and lowered-state rule
must be specified from the actual field. If a tag is visible in both positions,
presence alone is insufficient. A ty threshold also changes with camera range,
height and pitch, so any such test needs a controlled viewing pose or calibrated
3D geometry. Moving hive tags must not be blindly treated as stationary field-map
landmarks for robot localization.

## Gather the tag data

Select **Hive Vision Setup** on the Driver Station. It requires only `limelight`
and does not map or command motors or servos. D-pad UP/DOWN selects pipeline
0-9 in INIT or after START. It waits for a new result matching the requested
pipeline and ignores stale results. Telemetry lists each detected tag's ID,
tx, ty, area, and camera-space 3D position when available.

From the intended shooting position, record each hive when down and up:

| Hive | Pipeline | Tag ID | Down tx/ty | Up tx/ty | Visible only when down? |
| --- | --- | --- | --- | --- | --- |
| To configure | | | | | |

Correct tag size and appropriate 3D pipeline settings are needed for meaningful
3D distances. The setup mode does not decide which hive is down or enable firing.

## Inspect the uncertain device types

Select **Shooter Hardware Setup**. It lists configured types for `intake`,
`shooter`, `turret`, `hood`, and `blocker`, and reads motor encoder ticks and
velocity where available. It sends no motor power, servo position, or encoder
reset commands. Missing devices are reported rather than required.

Compare the reported type with the physical hub port and wiring. Configuration
type alone does not prove what device is connected. A servo's displayed last
command is not an angle measurement, and encoder numbers alone do not prove
there is a connected encoder or a valid home position. Do not interpret a
90-degree blocker movement as `setPosition(90)`; FTC positional servos use
normalized values from 0 to 1, with actual movement determined by the servo,
PWM configuration, and linkage calibration.

## Remaining shooting inputs

Before automatic firing, establish: target pipeline/tag mapping and down-state
test; camera-to-turret geometry; turret home and travel limits or direct visual
feedback arrangement; hood usable endpoints and gearing; shooter direction,
encoder ticks per revolution and working shot speed; blocker calibration and
servo travel; how intake feeds the shooter; spin-up, feed and blocker timing;
and a known working shot distance/hood setting. No firing power, turret limits,
or blocker/hood positions have been guessed.

The planned Ivy sequence is: block transfer, find a fresh confirmed down hive,
aim and spin up, open/feed only while aim and speed are ready, re-block/stop,
then collect again when estimated capacity allows. The existing 20-second
scoring/collection cutoff and final 10-second parking window remain the target
schedule for the integrated autonomous.

References: [FTC Limelight programming](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming)
and [3A setup](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a).
