# Recovered robot configuration

`Main.xml` restores the complete Main configuration read from the
connected Control Hub before its device entries disappeared on October 30, 2026.
It preserves the hub addresses, motor ports/types/names, blocker, transferColor,
Pinpoint, IMU and Limelight address from that earlier snapshot. It does not
guess ports for hardware absent from the snapshot, such as the hood.

At the user's request, this restores `/sdcard/FIRST/Main.xml` directly.
`Main-before-restore.xml.txt` preserves the Hub's file just before restoration.
Select/activate **Main** again in Driver Station and re-INIT the OpMode to reload
the recovered mappings. No OpMode is started remotely.
The XML can also be copied to a Control Hub with Android Debug Bridge:

```powershell
adb push TeamCode/robot-config/Main.xml /sdcard/FIRST/Main.xml
```

When adding hardware, edit the existing configured file. Preserve its device
names and ports when scanning; saving a new/default controller configuration
can replace those mappings. The exact UI action that rewrote Main was not
observed. The remote diagnostic commands used before recovery were read-only.
