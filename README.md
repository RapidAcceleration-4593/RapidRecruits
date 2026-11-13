# Level 1.1 Debugging the Differential Drivetrain

In this level, you will practice debugging by finding and fixing three bugs in the differential drivetrain from Level 1.0.

## Background Knowledge

Not every bug stops the code from compiling. Some bugs compile cleanly but make the robot behave incorrectly. Both kinds count as bugs, so you will need to check both that the code builds and that the robot drives the way it should.

Debugging is a skill that takes practice. A good approach is to change one thing at a time and rerun the simulation after each change. That way, you always know which change caused the result you see.

## Important Files

- [Robot.java](./src/main/java/frc/robot/Robot.java) - robot lifecycle, teleop control, and autonomous routine.
- [Drivetrain.java](./src/main/java/frc/robot/Drivetrain.java) - motor configuration, drive logic, and odometry.
- [Constants.java](./src/main/java/frc/robot/Constants.java) - tunable drivetrain constants such as speed limits.

**Note:** Classes related to simulation are clearly labeled (by name or by package).
Understanding them is not required for this course.

## Instructions

There are three bugs somewhere in the code.
Your job is to find and fix them.
The bugs are relatively easy to find, and you only need basic Java knowledge and what you learned in Level 1.0 to fix them.

### Finding and Fixing the Bugs

1. Start the simulation *(refer to level-1.0 for details)*.

2. If you get a compiler error, read it carefully.
It tells you which file and line the problem is on, and usually what kind of problem it is.
Compiler errors mean the code is written incorrectly, and are fairly easy to fix.

3. Once the code compiles, identify a behavior that does not work properly.
Think about which part of the code controls that behavior.

4. Inspect that code for anything that looks wrong.
Work backward from the behavior to figure out what must be wrong in the code.

5. Use the debugger if you want to.
The Java debugger launches automatically when you simulate the code.
It takes some skill to use, but can be very helpful for tricky bugs.
A tutorial on the VS Code debugger can be found [here](https://youtu.be/R1F5ihQKL4U?si=Y4FiWO3dgiiTNTfp&t=291).

6. Fix one bug at a time, then rerun the simulation to confirm the fix and check for remaining bugs.

## Next Section

Once the code compiles and the robot drives correctly in both Teleop and Autonomous, continue to the next level.

To move on, execute the commands below:

```bash
git restore .
git checkout level-1.2
```
