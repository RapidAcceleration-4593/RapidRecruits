# Level 2.2 Integrated Closed Loop Control

In this lesson, you will explore how to apply feedback and feedforward concepts to the closed loop controllers present on SparkMax motor controllers.

## Background Information
The main loop of a standard roboRIO program executes at 50 times a second (50 Hz).
For feedback control, this is not always sufficient.
Additionally, every 20ms the robot must read the encoder value, run a calculation, and send the result over the CAN bus to the SparkMax.
This can saturate the CAN bus.

Fortunately, we can attach encoders directly to the SparkMaxes.
Then, the SparkMax can read the encoder, perform the calculation, and update the voltage.
At 1000 Hz.

SparkMaxes aren't limited to PID either.
They can also use **feedforward control**, which was discussed several sections ago.
In essence, **FF** control predicts how much voltage will be needed to spin the motor at a certain velocity, without reference any feedback sensor.
It does this by modeling the *dynamics* of the system.

The most common feedforward is SVAG FF (Static Velocity Acceleration Gravity).
* The static term is how many volts are required to get the motor *juuust barely* moving.
In other words, the voltage to overcome static friction.

* The velocity term is how many volts it takes to run the motor at a constant velocity.

* The acceleration term is how many volts it takes to accelerate the motor at a given rate.

* The gravity term is used for arms and elevators, and represents the voltage required to counteract gravity.
The SparkMax uses two different constants, Kg and Kcos.
Kg is used for elevators, where the pull of gravity is constant.
Kcos is used for arms, where the pull of gravity is proportional to the cosine of the angle of the arm.

Notice that **none of these terms rely on feedback from an encoder.**
They are solely properties of the setpoint.

For more information, read the [REVLib docs](https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control).

## Important Files
[ArmSubsystem.java](./src/main/java/frc/robot/ArmSubsystem.java)

## Instructions
### Tuning Ks
1. Open Elastic.
Click Add Widget.
Drag and drop "Tuning/ManualMode" and "Tuning/Voltage" onto the grid.

2. Set ManualMode to true.

3. Gradually increase Voltage until the arm begins to move.
Back off.
Try to find the exact voltage where movement is just about to begin.



## Next Section
To move on to the next level, execute the commands below:
```
git restore .
git checkout level-3.0
```
