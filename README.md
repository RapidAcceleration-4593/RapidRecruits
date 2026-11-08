# Level 2.1 Creating an Arm Subsystem 

In this lesson, you write an arm subsystem which uses a PID controller to drive a motor.

## Important Files
[ArmSubsystem.java](./src/main/java/frc/robot/ArmSubsystem.java)

## Instructions
### Part A. Writing the Code
1. Create all the necessary class fields for the motor controller, encoder, and anything else you need.

2. Initialize the `ArmSimulation` in the class constructor, **using the default parameters**.
We need the simulation so that your code can actually do something.

3. Write the incomplete methods.
Intellisense is your friend.
Use autocomplete to help you find the method names and parameter types on the `Encoder`, `SparkMax`, and `PIDController`.

4. Tune the PID so that the arm behaves nicely.

5. Show your work to a senior programmer.


## Next Section
To move on to the next level, execute the commands below:
```
git restore .
git checkout level-2.2
```
