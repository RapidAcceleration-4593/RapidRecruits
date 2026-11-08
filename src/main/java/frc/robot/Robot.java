// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
    private int position = -1;
    private ArmSubsystem arm;

    // A joystick to read the trigger from.
    private final XboxController controller = new XboxController(0);

    public Robot() {
        arm = new ArmSubsystem();
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
            var params = ArmSimulation.kDefaultParams; 
            double setpoint =
                    params.minAngleRads() + ((double)position) / 4 * (params.maxAngleRads() - params.minAngleRads());
            arm.setSetpoint(setpoint);
        }

        arm.periodic();
    }

    @Override
    public void simulationPeriodic() {
        arm.simulationPeriodic();
    }
}
