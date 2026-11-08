// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** This is a sample program to demonstrate how to use a state-space controller to control an arm. */
public class Robot extends TimedRobot {
    private final Encoder encoder = new Encoder(0, 1);
    private final SparkMax motor = new SparkMax(0, MotorType.kBrushless);
    private final PIDController pid = new PIDController(1, 0.0, 0.1);
    private final ArmSimulation sim = new ArmSimulation(encoder, motor);
    private int position = 0;

    // A joystick to read the trigger from.
    private final XboxController controller = new XboxController(0);

    public Robot() {
        // We go 2 pi radians in 1 rotation, or 4096 counts.
        encoder.setDistancePerPulse(Math.PI * 2 / 4096.0);
    }

    @Override
    public void teleopPeriodic() {
        int oldPos = position;
        if (controller.getAButton()) {
            position = 1;
        } else if (controller.getBButton()) {
            position = 2;
        } else if (controller.getXButton()) {
            position = 3;
        } else if (controller.getYButton()) {
            position = 4;
        } else {
            position = 0;
        }

        if (oldPos != position) {
            double setpoint = ((double) position) / 4 * Math.PI / 2;
            pid.setSetpoint(setpoint);
        }

        double voltage = pid.calculate(encoder.getDistance()) * 12;
        SmartDashboard.putNumber("EncoderReading", encoder.getDistance());
        motor.setVoltage(voltage);
    }

    @Override
    public void simulationPeriodic() {
        sim.simulationPeriodic();
    }
}
