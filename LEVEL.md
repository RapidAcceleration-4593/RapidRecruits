# Level 1.1 Differential Drivetrain

In this level, you will debug a basic differential drivetrain.
This level uses a TimedRobot architecture, in which there are no commands, only subsystems.
Each subsystem's periodic method is called, along with the Robot's periodic.

## Background Knowledge
A differential drivetrain consists one or more motors on each side of the robot.
By driving both motors in the forward direction, the robot moves forward.
By driving both motors in the reverse direction, the robot moves backward.
By driving both motors in opposite directions, the robot turns.

If one motor is driving faster than another, the robot will turn in a wide circle, in the direction of the slower driving motor.
This means that in order for the robot to turn while driving forward, it must not be driving at maximum speed.
If it is driving at maximum speed, than one motor cannot drive slower than the other, meaning it won't turn.

## Important Files
[Robot.java](./src/main/java/frc/robot/Robot.java)

[Drivetrain.java](./src/main/java/frc/robot/Drivetrain.java)

[Constants.java](./src/main/java/frc/robot/Constants.java)

[Main.java](./src/main/java/frc/robot/Main.java)

**Note: Classes related to simulation will be clearly labeled (either by name or by package).
Understanding simulation classes is not important for this course.**

## Instructions
### Part A. Running the Simulation
1. Run the simulation by pressing Ctrl+Shift+P then typing "WPILib: Simulate Robot Code".
Make sure that "Use Real Driverstation" is checked.

2. Open up AdvantageScope. Click File > Connect to Simulator > NetworkTables 4.
Once connected, open up the 3D Field tab.
Then, drag and drop *SimulationPose* from the side panel.
You should see a robot model appear in a corner of the field.

3. Run FRC Driver Station.
Ensure that you see both "Communications" and "Robot Code" as green.
Connect a controller, and ensure that Driver Station sees it.

4. Enable the robot on Driver Station.
Drive around with the robot, using the controller joysticks.

5. On Driver Station, switch the mode to Autonomous.
Press enable, and see what happens.

### Part B. Changing Drive Behavior
1. Go to `Robot.java` and delete the cosine compensation factor (`Math.cos(turnControl * 2 / Math.PI)`) found in `teleopPeriodic()`.

2. Run the simulation. How does the driving change? Is it easier to turn?

3. Now you should reset the code to its original state.
In the command line, run the command `git restore . `.
This restores your workspace to the original state.
Alternatively, navigate to the Source Control panel of VS Code and press the Undo button on the changed files.

4. In `Drivetrain.java`, change the inversion on `rightLeaderConfig` from `true` to `false`.
Simulate.
How does this affect the driving pattern?

5. Now, create a new `SparkBaseConfig` with `inverted(true)`.
Apply it to the `leftLeader`.
Simulate.
Now how does the robot drive?

6. Restore the workspace to the original state (See step B3).

### Part C. Creating an Auto
1. In the constructor of `Robot.java`, change the instantiation of `trajectory` to have a waypoint.
Do this by changing `List.of()` to `List.of(new Translation2d(-1, -1))`

2. Simulate the auto.
What changed?

3. Create your own trajectory.
Try to make it drive in a rectangle around the entire arena.

### Part D. Changing the speed.
1. Reset the workspace.

2. In `Constants.java`, change `kMaxSpeed` to `1.0`.

3. Simulate.
What changed?

4. In `Constants.java`, change `kMaxAngularSpeed` to `1.0`.

5. Simulate.
What changed?

6. Change `kMaxSpeed` to `100.0`.

7. Simulate.
Why do you think that it isn't 100x faster?
Why do you think that it is impossible to turn while driving forward?
How might this relate to cosine compensation?

8. Change `kMaxAngularSpeed` to `100.0`.

9. Simulate.

These examples demonstrate the importance of properly tuning physical constants to match reality.
Setting the maximum speed to 100 m/s doesn't mean it will drive that fast!