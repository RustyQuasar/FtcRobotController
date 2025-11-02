# Road Runner Tuning — Quick Reference

A compact reference for the key `MecanumDrive.PARAMS` variables. Each entry has a short description and a practical tuning tip.

### `kS`

Static-friction feedforward (voltage to overcome stiction). Call out the tiny extra voltage needed to get wheels moving from rest.

Tuning: Start at 0.0. If motors hesitate to start moving, add small increments (e.g., 0.02–0.1) until the bot reliably begins motion without a noticeable jump.

### `kV`

Velocity feedforward (voltage per unit wheel velocity). This scales how aggressively the robot converts target velocity into motor voltage.

Tuning: Begin around 0.3–1.0 for testing. If motion is jerky or too aggressive, reduce `kV`; if the robot undershoots target speeds, increase `kV`. Tune with a simple straight-line test.

### `kA`

Acceleration feedforward (voltage per unit acceleration). Compensates for inertia during rapid speed changes and helps keep tracking consistent on starts/stops.

Tuning: Start at 0.0. If you observe lagging or overshoot during accelerations or decelerations, add a modest `kA` (e.g., 0.05–0.3) and watch for reduced overshoot.

### `maxWheelVel`

Maximum wheel velocity used when generating trajectories (in inches/sec or your configured units). Limits how fast trajectories ask the wheels to spin.

Tuning: Set slightly below the physical maximum of your drivetrain under load. Reduce if the robot tries to move too fast or slips.

### `minProfileAccel` / `maxProfileAccel`

Path acceleration bounds used by the trajectory profile (minimum and maximum acceleration allowed during path generation).

Tuning: Use a moderate `minProfileAccel` to avoid abrupt starts, and set `maxProfileAccel` to a value the robot can safely follow. Reduce both to make motion smoother; increase them to make motion more responsive.

### `maxAngVel`

Maximum angular velocity (radians/sec) for turning trajectories.

Tuning: Set to a conservative value for initial tuning (e.g., ~1 rad/s). Increase gradually until turns are quick but still stable.

### `maxAngAccel`

Maximum angular acceleration (radians/sec²) used when planning turns.

Tuning: Keep this modest at first to prevent abrupt rotational jerks; raise it if turns feel sluggish.

### `axialGain`

Proportional gain for the forward/backward positional error (P term for axial position). Multiplied by position error to generate corrective velocity.

Tuning: Start small (0.5–1.5). Increase until steady-state position error reduces without oscillation. Lower if you see overshoot or jitter.

### `lateralGain`

Proportional gain for lateral (strafing) positional error.

Tuning: Tune similarly to `axialGain` but independently—strafe dynamics differ. Increase slowly until lateral error corrects promptly without wobble.

### `headingGain`

Proportional gain for rotational (heading) error. Controls how strongly the controller corrects orientation errors.

Tuning: Start around 1.0. If the robot oscillates while turning, reduce it; if turns are slow to correct, increase it.

### `axialVelGain`

Velocity feedback gain for axial motion (how strongly to follow commanded wheel velocity in addition to feedforward).

Tuning: Start low (0.2–0.6). Raise to improve velocity tracking; too high causes oscillation. Combine with `kV`/`kA` adjustments.

### `lateralVelGain`

Velocity feedback gain for lateral motion (strafing).

Tuning: Same approach as `axialVelGain`; tune while watching lateral velocity error during a strafe test.

### `headingVelGain`

Velocity feedback gain for heading (rotational velocity). Helps the controller track commanded angular velocity.

Tuning: Start small (0.2–0.6). Increase to tighten angular velocity tracking; back off if rotation becomes unstable.

## Practical tuning order (recommended)

1. Feedforward: get `kV` (and small `kS`) so the robot moves predictably on open-loop commands. Keep `kA` = 0 for initial tests.
2. Profiles: set safe `maxWheelVel` and `maxProfileAccel` so trajectories are achievable.
3. Position gains: tune `axialGain`, `lateralGain`, `headingGain` for stable position correction.
4. Velocity gains: tune `axialVelGain`, `lateralVelGain`, `headingVelGain` to reduce steady-state velocity errors.
5. Fine-tune `kA` if necessary to improve transient response.

Always change one parameter at a time, run a small repeatable test (straight line, strafe, and turn), and use telemetry (errors and computed powers) to guide adjustments.
