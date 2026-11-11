# Pedro Shooter Tuning

Deploy the app and select **Pedro Shooter Tuning** in the **Test** group.
Place the robot at the blue autonomous start, **(132, 109), facing 270°**, with
the turret straight ahead and no pollen loaded. Wait for Pinpoint **READY**
before START. Aim uses the hive at **(83, 83)**. If the turret pivot is offset
from the Pedro tracking point, set `PIVOT_FORWARD_IN` and `PIVOT_LEFT_IN` in
Panels before START. Both default to zero. Keep the hood at a fixed physical
setting and update `HOOD_SETTING` in Panels when you change that setting.

After START the intake and shooter run continuously. Use the left stick to move
and the right stick to turn. Drive to the first test spot near **(83.9, 108.6)**,
center the sticks, and wait for the turret and flywheel readings to settle.

- Between trials, **D-pad up/down** changes the shooter speed command by five
  percentage points and **left/right** changes it by one. The starting command
  is 65% of the configured motor speed, adjustable via `START_SPEED_PERCENT`.
- Hold **RB** just before feeding one pollen and release it after the shot.
  RB only records data. Feed through your normal intake.
- After releasing RB, mark the shot with **A** for hit, **B** for miss,
  **D-pad up** for high, or **D-pad down** for low. High and low count as misses.
  Press **X** to discard a trial. A trial saves as soon as you mark the result.

The log captures pose and measured flywheel speed when you press RB, the lowest
measured speed while held, movement, and aim error. Keep the robot still while
feeding. Repeat promising settings at the same pose and heading, then try nearby
poses and farther distances.

The Hub retains scored trials across OpMode restarts. On the robot network,
the analysis tool can download them directly and generate the tables and map:

```powershell
python TeamCode/tools/analyze_pollen_shots.py http://192.168.43.1:8080/shot-tuning.csv --out TeamCode/build/shooter-field-test
```

The output includes the raw trials, `shooting-poses.csv`, `shooter-speeds.csv`,
and `shooting-map.svg`. The default screening threshold is 90% with at least ten
pollen at a similar pose, heading, and speed for the same hood and pivot setup.
The tool excludes trials with more than two inches of movement, five degrees of
heading change, or five degrees of turret aim error.
Speed rows represent tested poses in six-inch distance bands. Points outside
tested poses and headings still need verification on the field.
