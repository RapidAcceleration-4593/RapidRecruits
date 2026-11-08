# Level 1.1 Debugging the Differential Drivetrain

In this level, you will debug the drivetrain we learned about in Level 1.0.

## Important Files
[Robot.java](./src/main/java/frc/robot/Robot.java)

[Drivetrain.java](./src/main/java/frc/robot/Drivetrain.java)

[Constants.java](./src/main/java/frc/robot/Constants.java)

**Note: Classes related to simulation will be clearly labeled (either by name or by package).
Understanding simulation classes is not important for this course.**

## Instructions

There is are three bugs somewhere in the code.
Your job is to find and fix them.
The bugs are relatively easy to find.
You only need basic Java knowledge and what you learned in Level 1.0 to fix them.

### Part A. Finding the Bug
1. Run the simulation.
If you get a compiler error, read it and figure out where the problem in code is.
Compiler errors mean you wrote the code wrong, and are fairly easy to fix.
Try running both auto and teleop.
What doesn't work?

2. Once you identify a behavior that doesn't work properly, think about which part of code is controlling that behavior.

3. Inspect the code for things that look wrong.
Try to work back what must be wrong in code for the robot to exhibit that behavior.

4. If you want to use a debugger, you can.
The Java debugger automatically launches when you simulate the code.
They require some skill to use, but can be quite helpful for tricky bugs.
A tutorial on the debugger in VS Code can be found [here](https://youtu.be/R1F5ihQKL4U?si=Y4FiWO3dgiiTNTfp&t=291).

Debugging is a difficult skill to learn, practice makes perfect!

## Next Section
Once the code compiles and works normally, continue on to the next section.

To move on to the next level, execute the commands below:
```
git restore .
git checkout level-1.2
```
