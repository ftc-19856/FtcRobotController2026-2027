# Red and blue path autonomous

Driver Station OpModes: **Red Path Wait Park** and **Blue Path Wait Park**.
Both use Pedro 3 and Ivy, with the user-provided Visualizer geometry.

Alliance-specific classes are `RedPathAuto` with `RedAlliancePaths`, and
`BluePathAuto` with `BlueAlliancePaths`. Shared sequence/path implementation
lives in the abstract `AlliancePathAuto` and `AllianceAutonomousPaths` bases.

| Alliance | Initial pose | Shooting pose | Park pose |
| --- | --- | --- | --- |
| Red | (9.7, 32.8, 90 degrees) | (57.8, 27.3, 90 degrees) | (9.7, 115, 90 degrees) |
| Blue | (132, 109, 270 degrees) | (83.9, 108.6, 270 degrees) | (135.409, 23.3501, 270 degrees) |

Coordinates are absolute Pedro field inches. Angles above are **Pedro degrees**,
created with `PoseFactory.degrees()`; do not pass degree values directly to a
`Pose` constructor that expects radians. Blue starts physically at 270 degrees
and keeps that heading; no initial 180-degree turn or heading teleport is used.

Ivy runs path 1 (straight line), waits for Pedro's endpoint completion, pauses
for **3000 ms**, then follows path 2 (the parking curve). Red's curve control
point is (15.2254, 37.9912); blue's is (131.9551, 91.8665). The control points'
exported zero headings do not change the constant heading on the parking paths.
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
