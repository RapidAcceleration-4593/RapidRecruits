# Level 2.0 PID

In this lesson, you will learn about feedback control and tune a PID (Proportional Integral Derivative) sensor.

## Background
In robotics we often need to make something reach a setpoint.
For instance, holding an arm at a position, or spinning a shooter wheel at a certain weight.
It is technically possible to calculate the exact voltage we need to apply to a motor to run a wheel at a certain speed.
This method is called **feedforward control**.
To control the wheel using only feedforward, we would need to know the exact moment of inertia of the wheel, as well as the efficiency of the motor and gearbox.
Even then, we would never know for sure if the wheel is running at the speed it is supposed to, and some factor we didn't model might throw everything off.
Instead, we use **feedback control**.
Feedback control uses the signal from a sensor (often an encoder) to adjust how fast the motor is driving.
Humans do something similar when driving a car.
We don't know exactly how far we need to press the pedal to drive a certain speed.
Instead, we watch the feedback from the spedometer and adjust our speed accordingly.
We usually look at three things to adjust our speed:

1. How far away we are from the setpoint.
The farther away we are from the setpoint, the harder we push the pedal.

2. How long we have spent approaching the setpoint.
The longer we have been attempting to approach the setpoint, the harder we push the pedal.

3. How quickly we are approaching the setpoint.
The faster we are approaching the setpoint, the more likely we are to overshoot, so we push the pedal less.

A **PID controller** uses the exact same strategies.
It stands for Proportional, Integral, Derivative, and has three constants, Kp, Ki, Kd, which can be tuned to change the controller's behavior.
In control theory, we use the term **error** to denote the difference between the setpoint and the current value.
The goal of the controller is to minimize the error, because that would mean that we reached the setpoint.

Imagine a `SparkMax` which is controlling an arm.
An encoder reads the angle of the arm.
In this case, the error would be the setpoint angle minus the encoder reading.
In the simplest case, we could set the motor voltage to +12 if the error is positive, and -12 if the error is negative.
This is called **bang bang** control, and is extremely effective for things like deploying intakes or running climbers.
However, in the case of an arm, gravity would be constantly pulling it down, and then the motor would kick in at +12 volts.
This would make it jitter significantly.

Instead, we could make the motor voltage higher the farther away it is from the setpoint.
That way, it would quickly drive most of the way, then slowly come to rest on the setpoint.
This is called **proportional gain**, and is P in PID.
The math is simple: $$voltage = K_p \times error$$
It would seem that we would want to set Kp as high as possible.
That way the arm could shoot as quickly as possible over to the setpoint before coming gracefully to a rest.
Except momentum exists.
If Kp is set too high, the arm will overshoot the setpoint, then come swinging back the other way before overshooting again.
This is called **oscillation**, and is the bane of the programmer.
The general strategy when tuning Kp is to set it as high as possible while keeping oscillation tolerable.

Properly tuning kP will get us 90% of the way to accurately reaching a setpoint.
However, in the case of the arm, we have a problem: gravity is pulling it down.
This means that in order to counteract the pull of gravity, the motor has to constantly be applying a voltage.
If all we have is Kp, the only way to generate a voltage is to have error.
If we have error, we are not at the setpoint.
To solve this problem, we can use **integral gain**.
Integrals are a concept from calculus, but can be easily understood.
Imagine the error is 10.0.
If Kp = 0.1, than the voltage supplied by proportional gain would be 1.0.
If Ki = 0.1, than at time t=0sec the integral gain would be 0.0.
At t=1sec the integral gain would be 1.0.
At t=2sec the integral gain would be 2.0.
The arm starts to move.
Imagine at t=3sec the error is only 0.5.
In that case, the increase in integral gain would increase by less, so it be closer to 2.5.
The arm finally reaches the setpoint.
The integral gain stops increasing, but is still a positive value.
The motor keeps driving up.
Now the error is negative, and the integral gain begins decreasing.
Eventually it hits zero, and then becomes negative.
The motor starts driving down.

Integral gain is extremely powerful: of P, I, and D, it is the only one which allows us to *exactly* hit the setpoint.
But, it is also *extremely prone* to oscillatory behavior.
In order to work properly, Ki should be kept very small so that Kp does most of the work.
That way, there are less oscillations.
Most of the time, Ki should be 0, which effectively turns off integral gain.

The D in PID stands for Derivative gain.
It acts as a brake on P and I, lowering the voltage applied if the arm is approaching the setpoint too quickly.
It is similar to how we back off on the gas pedal as we approach the speed limit.
When it works, derivative gain is extremely helpful in reducing oscillations.
However, it is usually a bit of a wildcard and there is no easy rule for tuning.
Try it, see if it improves dynamics.
If it doesn't, set it to 0.

## Instructions