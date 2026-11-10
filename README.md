# Kinematics-Robotics-Projects



Summer project I did to explore forward and inverse kinematics in 6-DOF robotic arms.

Two 6-DOF arms are driven by an Arduino through an SSC-32U servo controller. You give the code a target for the tool tip (how far out, how high, and what angle the last link points at) and it works out the servo angles that get the arm there.

The finished code is in [`Robot_Arms_Final`](Robot_Arms_Final/Robot_Arms_Final.ino). The earlier versions from the summer are in [`versions/`](versions/), unchanged, so you can see how the code got here.

## Contents

```
Robot_Arms_Final/        final sketch; open this one in the Arduino IDE
versions/                the summer versions in order, exactly as they were
tools/host_test/         test setup that runs the sketches on a PC (no arm needed)
```

## Using the final sketch

What the sketch assumes (change the tables at the top of the sketch if yours differs):

- Arduino pin 10 is RX and pin 11 is TX, wired to the SSC-32U at 9600 baud
- Servo channels: Arm 1 on 2, 3, 4, 5, 8, 9 and Arm 2 on 16, 17, 20, 21, 24, 25 (base, shoulder, elbow, wrist tilt, wrist rotation, gripper)
- Servos map 500 to 2500 us onto 0 to 180 degrees
- The Serial Monitor is at 9600 baud with the line ending set to "Newline" or "Both NL & CR"

Commands are one line each:

```
<arm> <radius> <height> <gamma>
```

| Value | Meaning |
|---|---|
| arm | 1 or 2 |
| radius | horizontal distance from the shoulder axis to the tool tip (cm) |
| height | vertical distance from the shoulder axis to the tool tip (cm) |
| gamma | angle of the last link above the horizontal (degrees) |

Example session (this is real output from the sketch):

```
Ready. Send: <arm 1-2> <radius cm> <height cm> <gamma deg>
> 1 20 20 45
OK arm 1: radius 20.03 cm, height 19.99 cm, gamma 44.9 deg
> 1 60 60 0
Target not reachable; arm not moved
> hello
Usage: <arm 1-2> <radius cm> <height cm> <gamma deg>
```

If the target cannot be reached, the arm stays where it is, and nothing is sent to the servos. On start-up, both arms go to 90 degrees on every servo.

## How the kinematics works

The base, wrist rotation, and gripper are not part of this. The planar model uses three joints: shoulder, elbow, and wrist tilt, with link lengths L1 = 14.5 cm, L2 = 18.25 cm, and L3 = 10 cm.

**Angles.** The servo angles are converted to link angles like this:

```
theta_S = shoulder servo angle
theta_E = 180 - elbow servo angle
theta_W = wrist tilt servo angle

theta_1 = theta_S - 90
theta_2 = theta_1 + theta_E
theta_3 = theta_2 + theta_W - 90
```

**Forward kinematics.** From the link angles to the tool tip:

```
radius = L1 cos(theta_1) + L2 cos(theta_2) + L3 cos(theta_3)
height = L1 sin(theta_1) + L2 sin(theta_2) + L3 sin(theta_3)
gamma  = theta_3
```

**Jacobian.** A 3x3 matrix that says how radius, height and gamma change when each joint moves. The rows must be in the same order as the outputs (radius, height, gamma) and the columns in the same order as the joints (S, E, W):

```
row radius: -L1 sin(t1) - L2 sin(t2) - L3 sin(t3) | -L2 sin(t2) - L3 sin(t3) | -L3 sin(t3)
row height:  L1 cos(t1) + L2 cos(t2) + L3 cos(t3) |  L2 cos(t2) + L3 cos(t3) |  L3 cos(t3)
row gamma:   1                                    |  1                       |  1
```

**Inverse kinematics** is done in two stages in `invKine()`.

1. Closed-form starting guess (`seedFromClosedForm`). Pull the tip back along link 3 to get the wrist point (x, y). Links 1 and 2 are then a normal two-link arm:

   ```
   cos(theta_E) = (x^2 + y^2 - L1^2 - L2^2) / (2 L1 L2)
   theta_1      = atan2(y, x) - atan2(L2 sin(theta_E), L1 + L2 cos(theta_E))
   theta_W      = gamma - (theta_1 + theta_E) + 90
   ```

   The elbow only bends between 0 and 180 degrees, so there is exactly one answer. If the wrist point is closer than 3.75 cm or farther than 32.75 cm from the shoulder, no joint angles can reach it and the move is rejected.

2. Jacobian refinement (`jacobianStep`). The guess gets rounded to whole microsecond pulses and clamped to the servo limits, so a few damped least squares steps clean it up:

   ```
   error e = [radius error, height error, gamma error (radians)]
   step dq = J^T (J J^T + lambda^2 I)^-1 e
   ```

   It repeats until the tip is within 0.10 cm and 0.5 degrees, with at most 100 iterations and no joint moving more than 0.5 rad per step. If the guess cannot be refined (for example the wrist would have to go past its limit), it tries again from the arm's current pose, and if that fails the move is rejected.

**Why not the plain transpose step?** The summer version used `dq = alpha * J^T e` with alpha = 0.01. That method only stays stable when alpha is below 2 / (largest eigenvalue of J J^T). For this arm that eigenvalue reaches about 2700, so alpha has to stay under roughly 0.0007. At 0.01 every joint ends up slammed against its limit. Even with a stable alpha it needs far more than 50 iterations (a separate check with alpha = 0.0005 reached the target in only 5 of 300 cases after 50). The damped step takes properly sized steps and the lambda term keeps it from blowing up near a singular pose (elbow fully straight or fully folded).

## Version history

The order below is how the code builds on itself, not file dates. Some channel numbers changed between versions (Arm 1 base and shoulder are on 0 and 1 in some and 2 and 3 in others), so check them against your wiring.

| # | Folder in `versions/` | What it adds or changes |
|---|---|---|
| 1 | `Old_Arms` | First sketch. Servo and arm tables (SSC-32U channel, pulse width, speed, time) and a `build()` that sends one command per servo. Does not compile as saved: the last line of `loop()` is missing a semicolon. |
| 2 | `Robot_Arms` | Adds the `ServoIndex` names, per-servo angle offsets and `moveServo(arm, servo, degree)`. Still prints to the USB serial instead of the SSC-32U. |
| 3 | `Robot_Arms_5-26` | Moves to SoftwareSerial on pins 10 and 11 for the SSC-32U. First `fwdKine()` (prints radius and height). Serial command to move one servo: `arm servo degree`. |
| 4 | `Robot_Arms_FWD-Kinematics` | Forward kinematics prints radius, height and gamma for both arms. Angles are measured so that height comes from the cos terms and radius from the sin terms. |
| 5 | `Robot_Arms_INV_Kinematics` | Closed-form inverse kinematics (law of cosines), using the same angle convention as 4. |
| 6 | `Robot_Arms_INV_Kinematics_Jacobian` | Iterative Jacobian inverse kinematics with `ArmPos`. Switches to radius from cos and height from sin. |
| 7 | `robot_arm_ik_fixed` | First round of bug fixes on 6 (see below). |
| 8 | `Robot_Arms_Final` | Final version, in the top folder. |

## What was wrong in 6 and 7, and what changed

Version 6 as written:

- The Jacobian rows for radius and height were swapped, so each error was multiplied by the wrong sensitivity
- The gamma error was in degrees while the rest of the step was in radians, and `degrees()` was added to a variable that was in radians
- `build()` packed both arms into a 256-byte buffer with `strcat`. The command is about 290 characters, so it overflowed
- Every `moveServo()` call sent the full command. One move meant 151 transmissions of about 290 characters, roughly 45 seconds at 9600 baud
- Each servo got its own `\r`, so the servos of an arm did not start together
- The 12.5 degree elbow offset was cut to 12 because `offset` was an `int`

Version 7 fixed those, but its step size (alpha = 0.01) was the real problem, as covered above. It also still sent the pose from `loop()` on every pass, and each resend can restart the servo's move and stretch it out. `parseInt` and `parseFloat` can block for about a second per number (the default Serial timeout), including when a newline left over from the previous command is still in the buffer. That second point comes from reading how the Arduino core behaves, I have not timed it on a board.

Version 8 (final):

- Closed-form guess plus damped least squares refinement instead of the transpose step
- The target is checked first. If it cannot be reached the arm does not move, the move is not sent, and an error is printed
- Joint angles keep their fractions. `moveServo()` takes a `double` and the stored pulse has 1 us resolution, instead of rounding to whole degrees
- The gamma error is wrapped to -180..180 so 359 and -1 degrees count as 2 degrees apart
- One command line per arm with a single `T`. The pose is sent once per move instead of on every pass of `loop()`, and only for the arm that moved
- Pulses are clamped to 500..2500 us after the offset is added, and offsets are rounded instead of truncated
- Serial input is read without blocking, one line at a time, with a usage message for bad input
- Link lengths and settings are constants at the top instead of being repeated in three functions
- The comment on the arm time now says milliseconds, which is what the SSC-32U `T` value is

Measured on a PC with the same 1000 random reachable targets for each version, starting from the home pose:

| | Version 6 (as written) | Version 7 | Final |
|---|---|---|---|
| Reached the target | 0 of 1000 | 0 of 1000 | 1000 of 1000 |
| Median position error | 45.3 cm | 32.1 cm | 0.013 cm |
| Worst position error | 80.5 cm | 77.5 cm | 0.052 cm |
| Worst gamma error | 179.9 deg | 179.8 deg | 0.13 deg |
| Bytes sent per move | 43,546 | 124 | 60 |
| Unreachable target | arm moves | arm moves | arm stays put |

Version 6 only ran in the test after its command buffer was made bigger, because the original overflows it. For the Jacobian step alone (no closed-form guess), the final settings solve 4881 of 5000 targets from the home pose in about 5 iterations. The closed-form guess and the fallback are what take it to 100%.

## Testing without the arm

`tools/host_test` compiles a sketch on a PC (needs `g++`) with a small stand-in for the Arduino functions, then checks the inverse kinematics against an independent copy of the forward kinematics.

```
bash tools/host_test/run_tests.sh Robot_Arms_Final/Robot_Arms_Final.ino 1000
bash tools/host_test/run_tests.sh Robot_Arms_Final/Robot_Arms_Final.ino 1000 --avr-float
bash tools/host_test/run_tests.sh Robot_Arms_Final/Robot_Arms_Final.ino --serial
```

- The first picks random reachable targets and reports how close the arm gets, how much is sent to the SSC-32U, and what happens with unreachable targets
- `--avr-float` uses 4-byte doubles like an Uno or Nano does
- `--serial` feeds commands to `loop()` and checks the parser (pieces arriving separately, bad input, long lines, and so on)
- The same script can run `robot_arm_ik_fixed` from `versions/` for comparison. The Jacobian version aborts in the test because of the buffer overflow described above.

This is not the real Arduino toolchain. It cannot catch things like flash or RAM limits or timing on the board, and it cannot tell whether the kinematics match the physical arm.

***Note: the implementation of Claude comes from consolidating and formatting the repo.***
***Additionally, retesting the code virtually was done through Claude.***
***All code was written by me***

## Limitations and next steps

- Not tested on the physical arms. The link lengths, angle zero points and servo directions come from the code as it was, so try small moves first.
- The planar IK only controls shoulder, elbow and wrist tilt. The base, wrist rotation and gripper stay where `moveServo()` last put them. Adding a serial command for them would bring back what version 3 could do.
- Some targets with gamma pointing down are rejected because the wrist tilt servo would need to go below 0 degrees. For example `1 30 5 -30` is refused, but `1 30 5 -20` works.
- The two arms are sent as separate commands, so they are not synchronized with each other, and Arm 1 and Arm 2 use different move times (1000 ms and 500 ms).
- `speed` in `ArmData` is not used yet. The SSC-32U `S` value could use it.
