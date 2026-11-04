# Panels

Panels is a Gradle dependency of the robot app (`com.bylazar:fullpanels:1.0.19`).
Android Studio downloads it during Gradle sync and includes it when deploying
the app to the Control Hub. No separate computer program is required.

## Open the dashboard

After deploying, connect your viewing device to the robot network (or the Pi
router with access to the Hub) and open **http://192.168.43.1:8001**.
Panels starts with the robot app. If it was previously disabled, use its
**Enable/Disable Panels** OpMode to enable it again.

## Robot telemetry

Add a **Telemetry** widget. The existing SDK readings are sent to both Panels
and the Driver Station in these modes:

- Pedro Pollen Shoot And Park, including Blue/Red Shoot Leave Park.
- Pedro Pollen Collect And Park.
- Search Intake And Return (the older intake/return mode).
- Hive Vision Setup.
- Fixed Turret Intake Shooter and Starter TeleOp.
- The Red/Blue path-and-wait modes based on AlliancePathAuto.
- Pedro Pinpoint Rotation Check.

For pollen tracking, INIT the desired vision mode and watch **Vision decision**,
**Last visible target**, **Last visible area (%)**, and **Target area (%) / frame
age (ms)** in the shooting/collecting auto. These are the same readings shown
on the Driver Station. Video alone does not establish that a target passed the
autonomous's range, area, or boundary checks.

## Limelight video and configuration

Add Panels' **Limelight Stream** widget for continuous video and its Limelight
controls for the camera dashboard. FullPanels includes the native Limelight
proxy, which forwards requests through the Hub to **172.29.0.1** (the address
already used by this robot). Keep the Limelight connected to the Control Hub.
The video widget uses this proxy directly and does not need the Camera Stream
widget or a second camera processor in the driving loop.

The camera dashboard is also reachable at **http://192.168.43.1:5801** through
the Panels proxy. The existing read-only viewer remains available at
**http://192.168.43.1:8080/limelight** while a vision mode is initialized.

Installation and plugin versions follow the
[official FullPanels documentation](https://panels.bylazar.com/docs/com.bylazar.fullpanels/).
Drive commands, mechanism commands, vision filtering, paths, and INIT movement
behavior are unchanged by this integration. Deploy and verify telemetry/video
on the actual Hub before relying on the dashboard during a run.
