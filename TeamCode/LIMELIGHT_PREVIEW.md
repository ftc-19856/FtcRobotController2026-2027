# Limelight camera preview

Panels is also available at **http://192.168.43.1:8001** after deploying the
updated app. Add its **Limelight Stream** widget for live video and **Telemetry**
for the OpMode readings. See [PANELS.md](PANELS.md) for setup and supported modes.

Deploy the updated app, then INIT one of these modes:

- Search Intake And Return (the legacy intake/return autonomous).
- Pedro Pollen Collect And Park.
- Red/Blue Shoot Leave Park or Pedro Pollen Shoot And Park.
- Hive Vision Setup (no mechanism/drive motors are mapped by this setup mode).

## Driver Station

While in INIT, open the three-dot menu and select **Camera Stream**. Tap the
image to refresh it. This is the SDK's native on-demand preview; ordinary Driver
Station telemetry cannot embed continuously updating video. Close Camera Stream
to return to the normal START/gamepad controls.

## Continuous video with camera telemetry

On a device connected through the robot/Pi-router network, open:

**http://192.168.43.1:8080/limelight**

This page displays continuous Limelight video plus pipeline, validity, area,
raw target angles, frame age and connection status. Shooting/collecting autos
also publish their target/lock decision. It uses the existing robot web server
to relay the camera feed, so the camera stays plugged into the Control Hub.
Refresh the page after reinitializing/changing an OpMode. Stop closes preview
connections. At most two camera preview connections are allowed at once.

The SDK/browser preview below needs no camera tuning or robot hardware-configuration
changes. Panels is a separate project dependency. The camera address is `RobotConfig.LIMELIGHT_IP`, currently
172.29.0.1, matching Main.xml. Preview requests run outside the driving loop;
no images are fetched until a DS/browser viewer requests them. Browser viewers
consume Wi-Fi bandwidth while open.

These changes add viewing support; they do not change the older autonomous's
drive or pickup calculations. Search Intake And Return still contains its old
camera measurements and horizontal-camera assumptions, which differ from the
current portrait mount. Use the view/readings to verify the remaining tracking
issue before changing those assumptions.
