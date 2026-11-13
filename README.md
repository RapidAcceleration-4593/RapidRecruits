# Level 1.0 Differential Drivetrain

In this level, you will learn about a basic differential drivetrain.
This level uses the `TimedRobot` architecture: there are no commands, only subsystems.
Each subsystem's `periodic()` method runs every cycle, along with the robot's `periodic()` method.

## Background Knowledge
A differential drivetrain consists of one or more motors on each side of the robot.
Driving both sides forward moves the robot forward.
Driving both sides in reverse moves the robot backward.
Driving the sides in opposite directions turns the robot in place.

If one side drives faster than the other, the robot turns in a wide arc toward the slower side.
This means the robot cannot turn while driving forward at maximum speed: neither side can slow down, so it cannot turn.
To turn while driving forward, leave some speed in reserve.

## Important Files
- [Robot.java](./src/main/java/frc/robot/Robot.java) - robot lifecycle, teleop control, and autonomous routine.
- [Drivetrain.java](./src/main/java/frc/robot/Drivetrain.java) - motor configuration, odometry, and drive logic.
- [Constants.java](./src/main/java/frc/robot/Constants.java) - tunable drivetrain constants such as speed limits.

**Note:** Classes related to simulation are clearly labeled (by name or by package).
Understanding them is not required for this course.

## Instructions
### Part A. Running the Simulation
1. Start the simulation by pressing Ctrl+Shift+P and typing "WPILib: Simulate Robot Code".
Make sure "Use Real Driverstation" is checked.

2. Open AdvantageScope and connect with File > Connect to Simulator > NetworkTables 4.
Once connected, open the 3D Field tab and drag and drop *SimulationPose* from the side panel.
A robot model should appear in a corner of the field.

3. Run FRC Driver Station.
"Communications" and "Robot Code" should both be green.
Connect a controller and confirm that Driver Station detects it.

4. Enable the robot and drive around using the controller joysticks.

5. Switch the mode to Autonomous, press Enable, and observe what happens.

### Part B. Changing Drive Behavior
1. In `Robot.java`, delete the cosine compensation factor (`Math.cos(turnControl * 2 / Math.PI)`) in `teleopPeriodic()`.

2. Back in WPILIB, observe the UI shown below

    ![Screenshot](assets/image.png)

    This Allows you to control the running of the simulation
    - Pause: Toggles the running of the simualtion
    - Restart: Restarts the simulation
    - Stop: Stops the simulation
    - Hot code replace (lightning bolt): Allows you to update changed code without restarting

    *Pretty revolutionary stuff, I know*

3. Restart the simulation and switch the Driver Station mode to Teleop. How does the driving change? Is it easier to turn?

4. Reset the code to its original state.
In a terminal, run `git restore .`.
This restores your workspace to the original state.
Alternatively, open the Source Control panel in VS Code and discard changes on the modified files.

5. In `Drivetrain.java`, change the inversion on `rightLeaderConfig` from `true` to `false`, then simulate.
How does this affect the driving pattern?

6. Create a new `SparkBaseConfig` with `inverted(true)` and apply it to `leftLeader`, then simulate.
Now how does the robot drive?

7. Restore the workspace to the original state (see step B3).

### Part C. Creating an Auto
1. In the `Robot()` constructor, the trajectory starts with one waypoint: `List.of(new Translation2d(-1, -1))`.
Change the waypoint's coordinates, or add another `Translation2d` to the list.

2. Simulate the auto. What changed?

3. Create your own trajectory.
Try to make it drive in a rectangle around the entire field.

### Part D. Changing the Speed
1. Reset the workspace (see step B3).

2. In `Constants.java`, change `kMaxSpeed` to `1.0`.

3. Simulate. What changed?

4. In `Constants.java`, change `kMaxAngularSpeed` to `1.0`.

5. Simulate. What changed?

6. Change `kMaxSpeed` to `100.0`.

7. Simulate.
Why isn't it 100x faster?
Why is it impossible to turn while driving forward?
How might this relate to cosine compensation?

8. Change `kMaxAngularSpeed` to `100.0`.

9. Simulate.

These examples demonstrate the importance of properly tuning physical constants to match reality.
Setting the maximum speed to 100 m/s doesn't mean the robot will drive that fast!

## Next Section
To move on to the next level, run:
```bash
git restore .
git checkout level-1.1
```
