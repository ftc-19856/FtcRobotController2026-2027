# Shoot, seek four, shoot again, park

Use **Red Shoot Leave Park** or **Blue Shoot Leave Park**. These modes now share
the working Pedro pollen seeker with **Pedro Pollen Shoot And Park**. The plain
Red/Blue Path Wait Park modes and intake-only autonomous are unchanged.

1. Drive to the shooting pose, then shoot the four preloads.
2. Seek pollen with the blocker closed and intake/transfer running.
3. Count each stable **yellow -> not yellow** transition at `transferColor`.
   Stop and brake after a counted pickup; stop the intake at four.
4. Return to the shooting pose and heading, shoot, and repeat.
5. At **25 seconds**, interrupt collection, returning, or shooting; close the
   blocker, stop the mechanisms, and drive to the alliance park coordinates.
   At **30 seconds**, stop driving. A partial batch stays aboard when parking.

The fixed turret must be aimed at the hive at the shooting pose. Red starts and
shoots at (9.7, 32.8, 90 Pedro degrees). Blue starts at (132, 109, 270 Pedro
degrees), then drives to (83.9, 108.6, 270 Pedro degrees) to shoot both preloads
and subsequent batches. It waits for the drive and heading to settle before shooting.
Blue leaves the turret motor at zero power in RUN_WITHOUT_ENCODER with BRAKE;
it does not actively hold, aim, or rotate the turret, including while shooting.
Position the turret manually before running. Red and TeleOp retain their existing
encoder hold. The blue chassis still faces the specified 270-degree shooting heading.
The established park endpoints are unchanged. Since seeking can end anywhere,
parking uses a curve from the current pose through the original alliance
control point. Blue uses (131.9551, 91.8665); red uses (15.2254, 37.9912).
Blue inserts clearance waypoints around the hive as described below.
These are Bezier control points, not waypoints the robot necessarily passes
through. Verify field clearance and that the return fits five seconds.

**Blue side boundary:** Blue Shoot Leave Park restricts the Pedro robot-center
coordinate to X > 70. Search and pickup centers are planned at X >= 72 to leave
an extra two-inch buffer. The physical field walls still use the 18x18 robot's
turning clearance. Targets across X = 70 are ignored. Return paths and parking waypoints
stay on the blue side; the parking control point is also on that side. A reported center X <= 70 stops the run. This
internal boundary applies to the center; turning alone cannot trigger the stop.
Red's collection region is unchanged.

## Transfer color sensor

Configure the REV V2 color sensor as **transferColor**. Distance is unused.
Watch **Transfer RGB**, **Transfer yellow**, and **Pollen held** in telemetry.
Yellow must be true with pollen in front of the sensor and false once it clears.
The yellow and not-yellow readings must each persist for 40 ms. The counter
only runs while collecting; outgoing preloads and shooting do not add pollen.
Touching pollen without a visible gap cannot be distinguished by this sensor.

`YELLOW_MIN_CHANNEL` (initially 20) and `YELLOW_BLUE_RATIO` (initially 1.5) in
`PedroPollenShootAndPark` define yellow: both red and green must meet the minimum
and each exceed blue times the ratio. These are untested starting values; check
real readings before running the autonomous. They are shared by both alliances.
Pickup attempts no longer increase the count without a sensor passage.

Shooting retains the existing timed settings: 1.5-second spin-up and 3-second
feed. The count is reset after that feed finishes; clearing the shooter is still
a timed assumption, not verified by the transfer sensor. Hood and turret aiming
are unchanged. There is no hive AprilTag targeting or garden unloading.

## Portrait Limelight mounting

Viewed from in front of the robot looking at the lens, the camera's original
bottom is now on the right. The Pedro pollen modes convert raw camera angles
into robot-relative angles before ranging: horizontal = raw ty, vertical = -raw tx.
The web stream rotation does not replace this targeting correction.
Camera optical-center height remains assumed to be 8.5 inches, 7.5 inches forward
of robot center, with no lateral offset and a level optical axis. Rotating the
camera in its own plane does not introduce a downward pitch. Update those
measurements if the remount changed them.

With a floor pollen directly ahead, corrected horizontal should be near zero
and corrected vertical should be negative. Moving the pollen toward the robot's
right should make corrected horizontal positive. Verify this before driving.

## Stop diagnostics

Telemetry shows Stop reason when the routine ends: BLUE X BOUNDARY, INVALID POSE,
30 SECOND STOP, or ROUTINE COMPLETE. During RETURN, distance and heading error to
the shooting pose are shown. A boundary stop cancels the run, so it will not
collect or park afterward. The blue internal boundary stop uses center X only, so turning in place near
the shooting pose cannot trigger it just because the robot's corners rotate.

### Verify the installed revision

Before starting, Driver Station telemetry must show **Auto revision: park-curve-vision-r4**.
If it does not, the robot is running a different build. When the routine stops,
**STOP X / Y (in)**, **STOP heading (deg)**, **STOP phase**, and **STOP seconds**
preserve the first stopping sample; the live Pose line can keep changing afterward.
For BLUE X BOUNDARY with this revision, the saved STOP X must be <= 70.000.
A displayed Pose heading such as 4 is in radians (about 229 degrees); the new STOP
heading is explicitly in degrees.

### Check position drift during rotation

Run **Pedro Pinpoint Rotation Check** under TeleOp/Test. It reads the exact
`Constants.localizerConfig` used by autonomous, including pod type, directions,
offsets and units. It does not map or command any drive or mechanism motor/servo.
This is separate from the legacy Pod Offset Calibration mode, which uses a
different configuration.

Keep still during INIT. Mark the robot's physical center on the floor, press
START, then rotate it by hand around that mark with both pods touching the floor.
Pause at 90 degrees and at 180 degrees. Record Turn since START, X change,
Y change and Maximum position drift. A stationary center should report little
translation. Do not lift the pods or rotate around one wheel.

The existing Pinpoint Tuner's Offsets Identification procedure assumes an
in-place 180-degree counterclockwise turn and calculates offsets from the final
translation divided by two. Generated values still need this independent check
if the calibration pivot moved or the turn differed from 180 degrees. This
diagnostic does not overwrite the tuned values.

Revision **rotation-offsets-r2** also displays the starting heading and candidate
corrected X/Y pod offsets when paused between 170 and 190 degrees of rotation.
It accounts for the actual rotation angle, transforms displacement into the
initial robot frame, and corrects the existing offsets instead of assuming zero
offsets. Record the candidates while the robot is still. They are not applied
automatically and are only valid if the physical center stayed on its mark and
pod directions/resolution are correct.

The reported candidates **X pod = 5.93 inches, Y pod = -6.65 inches** have been
applied to Pedro's `Constants.localizerConfig`. Pod type remains four-bar and
encoder directions remain as tuned. Deploy and repeat the centered rotation
check: X/Y change should stay near zero. These offsets are shared by all Pedro
modes; the legacy `RobotConfig` offsets are separate.

## Visible target without an approach

The camera LED alone does not indicate whether a target passed robot-code checks.
During INIT or running, read **Vision decision**, **Last visible target**,
**Last lock result**, **Lock frames**, and the raw
and corrected camera angles. The most recent visible-target decision stays on
screen after detection is lost. Reasons identify pipeline mismatch, stale frames,
small target area, invalid ground angle/range, a projected point outside the
field, or an unreachable pickup pose. Three distinct consistent camera frames
are still required before approaching. The extra robot-code minimum target
area is **0.01%**, lowered from 0.1% after valid detections at approximately
three feet were rejected. **Last visible area (%)** preserves the detected
area after the camera loses the target. Pipeline validity, frame freshness,
ground-angle/range, field/boundary and target-consistency checks remain active.

## Blue parking around the hive

The reported obstruction is modeled as X=94.5..95.5, Y=50..90 inches. Blue's
parking planner reserves the robot's half-diagonal plus two inches around it.
It uses the supplied parking curve if that entire curve clears this buffer.
Otherwise, it takes Pedro line segments around the nearer clear end, followed
by a curve using the original (131.9551, 91.8665) control point to the original
park endpoint. From the shooting pose, this goes above the structure and across
to about (110.23, 104.73) before the final curve.

If collection ends inside the conservative turning buffer but the current robot
footprint is still clear of the structure, the first leg moves directly away
with heading held fixed before allowing a turn. If no clear escape is available,
the routine stops with NO CLEAR PARK ROUTE. The five-second parking budget and
30-second cutoff remain unchanged. Route clearance depends on correct
localization and the reported obstacle dimensions.

This obstacle routing applies to Blue Shoot Leave Park's parking phase. Intake
approaches and returns to shoot retain their existing straight paths.
