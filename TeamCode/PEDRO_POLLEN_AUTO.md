# Pedro pollen intake and park

Select **Pedro Pollen Collect And Park**. The current task is intake only;
shooter, turret, and hood are not mapped or commanded by this autonomous.
Uses Pedro 3.0.1 and Ivy Pedro integration 1.1.1. The supplied Foresight tuning
and the existing return-to-park path are preserved.

## Before START

The start and default park pose is **(132.4, 9.8, 90 degrees)** in absolute
Pedro field coordinates. The robot footprint is **18 x 18 inches**. During INIT,
use gamepad 1 D-pad UP/DOWN to set the actual starting pollen count (0 through
4). The default match preload remains four. For an empty-robot intake test,
select zero. A full robot will not collect more pollen.

## Collection

Collection lasts up to 20 seconds, followed by a 10-second park budget. Instead
of adjusting manual steering power for every camera reading, collection now
uses the tuned Pedro follower for its approach:

1. Move the robot center inward if needed to make room for an 18-inch square
   robot to rotate inside the 144-inch field. The center stays at least the
   half-diagonal plus one inch from an edge during collection. From the starting
   corner this is a short movement before the first scan.
2. Rotate at a requested 70 degrees/second using Pedro's heading control with
   zero translation command. Scanning and target alignment use scaled holds
   with translation scaling zero and heading scaling one. This prevents XY
   position corrections from mixing into a turn and creating a wheel pivot.
   The regular pickup and parking paths retain their tuned position control.
3. On a visible pollen detection, stop the scan, settle for 0.12 seconds, then
   lock the median field position from three distinct camera frames agreeing
   within four inches. An inconsistent or lost detection resumes scanning.
4. Use Pedro to face the locked target, then follow one straight path with the
   intake leading. Camera loss or angle noise near the ball cannot repeatedly
   stop, slow, or rebuild this path.
5. Stop the wheels in zero-power BRAKE for 0.35 seconds with intake running,
   estimate one pickup, then remain braked for another 0.25 seconds before
   rotating to find the next target. Briefly ignore the previous target's
   location to avoid immediately counting a lingering detection twice.

The intake is REVERSE, so positive power collects pollen. An approach which
times out before reaching the pickup pose is not counted. The pickup goal
assumes the intake mouth is nine inches forward of center and includes three
inches of overtravel; adjust `INTAKE_FORWARD_OFFSET` and `PICKUP_OVERTRAVEL`
after field testing. Targets too close to a wall to be reached with turn
clearance are skipped. There is no general obstacle avoidance.

There is no count sensor. Each completed approach and dwell adds one estimated
pollen, capped at four, and collection stops while full. This cannot verify
physical load: missed or multiple pickups can make the estimate inaccurate.
The robot waits until the 20-second park transition when full.

## Camera and parking

Hardware: `frontLeftMotor`, `frontRightMotor`, `backLeftMotor`,
`backRightMotor`, `pinpoint`, `intake`, `limelight`.
Pollen pipeline is `RobotConfig.POLLEN_PIPELINE_INDEX` (0).
Camera measurements: **8.5 inches high, 7.5 inches forward of robot center,
centered horizontally**. The camera is assumed level and forward-facing; pollen
diameter is 2.85 inches. Floor range comes from ty, lateral position from tx,
and Pedro's localized pose converts the result to field coordinates. Stale,
invalid, tiny-area, out-of-field and unreachable detections are ignored.

At 20 seconds Ivy cancels collection and switches off intake. Parking uses the
existing Ivy Pedro `follow`/`hold` commands and endpoint correction. It returns
to the starting pose by default, or the configured `PARK_X`, `PARK_Y`,
`PARK_HEADING` (inches, inches, radians). It stops on arrival or at 30 seconds;
SDK STOP also cancels the routine and immediately zeros drive/intake power.

Telemetry shows the collection phase and locked pickup pose. Verify camera
range, intake mouth offset, overtravel, turn clearance and scan/approach behavior
on the robot. Search telemetry also shows reported position drift during the
spin. Heading-only control prevents commanded translation but cannot correct
mechanical wheel slip or an inaccurate odometry tracking point. If drift remains,
verify motor mapping and the Pedro Pinpoint pod offsets relative to the chassis
center of rotation using the existing Pinpoint Tuner. The OpMode restores the
previous hold scaling settings on STOP.
This Pedro autonomous is separate from the existing
`SearchIntakeAndReturn`; merging it into master preserves that autonomous's code
and the existing legacy odometry correction.
