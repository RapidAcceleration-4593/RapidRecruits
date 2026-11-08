package frc.robot;

public class ArmSubsystem {
    
    // TODO: Create a brushless SparkMax and an encoder
    // TODO: Create and tune a PIDController
    private ArmSimulation sim;

    public ArmSubsystem() {
        // TODO: Initialize the ArmSimulation using the motor controller and encoder
        // Use the default simulation parameters.
    }
    
    public void setSetpoint(double setpoint) {
        // TODO: Use PIDController.setSetpoint()
    }

    public void periodic() {
        // TODO: Use PIDController.calculate() to find motor voltage
    }

    public void simulationPeriodic() {
        sim.simulationPeriodic();
    }
}
