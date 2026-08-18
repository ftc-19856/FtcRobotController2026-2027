# StarterBot setup

`StarterBotTeleOp` expects these names in the Robot Configuration:

| Configuration name | Device type | Port information provided |
| --- | --- | --- |
| `left_front_drive` | Motor | Control Hub port 3 |
| `right_front_drive` | Motor | Control Hub port 1 |
| `left_back_drive` | Motor | Control Hub port 2 |
| `right_back_drive` | Motor | Control Hub port 0 |
| `intake` | Motor | Expansion Hub port 0 |
| `left_intake_servo` | Continuous Rotation Servo | Control Hub servo port 1 |
| `right_intake_servo` | Continuous Rotation Servo | Control Hub servo port 0 |

The four drive motors and both intake servos use the Control Hub, while the
intake motor uses Expansion Hub motor port 0. Give each physical device the
name from the table when editing the configuration on the Driver Station.

The goBILDA intake uses two dual-mode speed servos in continuous-rotation mode.
Configure both as continuous-rotation servos. The TeleOp initializes both to
zero power and reverses the left servo so the two corner sweepers rotate
toward the center together.

## Driver controls

- Left stick: forward/backward and strafe
- Right stick left/right: turn
- Tap A: start the intake; tap A again to stop it
- Tap B: reverse the intake; tap B again to stop it
- Tap the other button to switch directions
- Press A and B together to stop the intake

For the first test, put the robot on blocks. If a wheel moves in the wrong
direction, correct that motor's direction in `StarterBotTeleOp`. If the entire
intake moves game elements outward when A is pressed, reverse the intake motor and
both intake-servo directions in code.
