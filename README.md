# Level 1.2 Odometry and Pose Estimation

In this level, you will learn about odometry, and see the effects of improper calibration on the accuracy of an auto routine.

## Background Knowledge
When creating autonomous routines, and even in teleop, it is often useful to know where the robot is on the field.
This can help us do things like shoot into a goal while moving, or drive to a certain place on the field to pick up an object.

## Important Files
[Robot.java](./src/main/java/frc/robot/Robot.java)

[Drivetrain.java](./src/main/java/frc/robot/Drivetrain.java)

**Note: Classes related to simulation will be clearly labeled (either by name or by package).
Understanding simulation classes is not important for this course.**

## Instructions
### Part A. Viewing the Pose Estimate
1. Run the simulation.

2. In AdvantageScope, in the left panel find the field named *EstimatedPose* and drag and drop it into the Poses panel on the 3D Field.
You should see a green ghost robot in the same position as the simulated robot.

3. The simulated robot is where the robot "actually is", and is the ground truth in the simulation.
However, in real life we do not have access to where the robot actually is, and instead have to guess.
The estimated pose is our guess of where we are.

4. Drive around. You will find that the estimated pose quite accurately matches the real pose.

### Part B. Making the Pose Estimate Worse
1. In `Drivetrain.updateOdometry()`, multiply the encoder readings of the left and right encoder by 0.8.

2. Drive the robot around.
What happened?

3. Run an auto.
Notice that while the real robot doesn't follow the original path, the pose estimate does.

4. Reset the workspace using `git restore . `.

5. In `Robot.java`, change the argument to the constructor of `SimulatedDrivetrain` from `false` to `true`.
This enables wheel slip simulation.

6. Simulate.
Notice how over time the estimated pose drifts from the actual pose.
If this goes on for long enough, the estimated pose is useless.
This is why many pose estimators use cameras (or Meta Quests) to provide vision estimates.
These are often less accurate on a frame by frame basis, but they don't drift over time.

## Next Section
To move on to the next level, execute the commands below:
```
git restore .
git checkout level-1.3
```