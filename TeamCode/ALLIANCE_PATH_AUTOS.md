# Red and blue path autonomous

Driver Station OpModes: **Red Path Wait Park** and **Blue Path Wait Park**.
Both use Pedro 3 and Ivy, with the user-provided Visualizer geometry.

Alliance-specific classes are `RedPathAuto` with `RedAlliancePaths`, and
`BluePathAuto` with `BlueAlliancePaths`. Shared sequence/path implementation
lives in the abstract `AlliancePathAuto` and `AllianceAutonomousPaths` bases.

| Alliance | Initial pose | Shooting pose | Park pose |
| --- | --- | --- | --- |
| Red | (8.8061, 32.1239, 90 degrees) | (53, 20, 90 degrees) | (11.3177, 96.0856, 95.8081 degrees) |
| Blue | (132, 109, 270 degrees) | (87, 120, 270 degrees) | (128.6823, 43.9144, -84.1919 degrees) |

Coordinates are absolute Pedro field inches. Angles above are **Pedro degrees**,
created with `PoseFactory.degrees()`; do not pass degree values directly to a
`Pose` constructor that expects radians. Blue starts and shoots at 270 degrees,
then interpolates to the nearly identical parking heading.

Ivy runs path 1 (straight line), waits for Pedro's endpoint completion, pauses
for **3000 ms**, then follows path 2 (the parking curve). Red's cubic uses
(0.0375, 46.4038) and (14.0673, 69.0546). Blue's cubic uses (139.9625, 93.5962) and
(125.9327, 70.9454). Control point headings do not set the robot heading.
Pedro remains active during the pause to hold the shooting position.

This version only **pauses** in the shooting phase. It does not map or command
intake, shooter, turret, hood, blocker, or Limelight hardware. Shooting settings
have not been calibrated, so no firing commands are assumed. It requires only
the configured Pedro drivetrain and Pinpoint hardware.

Parking begins after the 3-second pause, rather than waiting for a fixed
20-second transition. A 30-second global cutoff and SDK STOP cancel the routine
and immediately stop the drivetrain. The route endpoints and curves are
preserved as supplied; verify physical clearance for the 18x18 robot on the field.

Existing intake autonomous, legacy autonomous, and tuned Foresight constants
are unchanged. Future mechanism commands can replace the shooting-wait phase
without changing either route.
