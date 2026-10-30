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
parking uses a line from the current pose rather than the old curve with a
fixed starting point. Verify field clearance and that the return fits five seconds.

**Blue side boundary:** Blue Shoot Leave Park keeps the entire 18x18 robot above
Pedro X = 70. Search and pickup centers stay at or above about X = 83.73, allowing
the robot's half-diagonal plus one inch of clearance when turning. Unreachable
pollen near or across the boundary are ignored. Return and park lines have both
endpoints on the blue side. A localization-based stop also stops all actions if
the reported center reaches the swept-footprint limit (about X = 82.73).
Red's collection region is unchanged. Physical clearance still depends on accurate
localization and braking; this is not a physical barrier.

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
