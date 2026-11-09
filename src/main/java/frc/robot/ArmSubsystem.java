package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ArmSubsystem {

    private final ArmSimulation.Parameters params = ArmSimulation.kDefaultParams;
    private final ArmSimulation sim;
    private final SparkMax motor = new SparkMax(0, MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();
    private double Kp = 10;
    private double Ki = 0;
    private double Kd = 1;
    private double kPositionConversionFactor = 2 * Math.PI; // Converts rotations to radians
    private double kVelocityConversionFactor = 2 * Math.PI / 60; // Converts RPM to rad/s

    public ArmSubsystem() {
        SparkBaseConfig config = new SparkMaxConfig()
                .apply(new ClosedLoopConfig().pid(Kp, Ki, Kd).feedbackSensor(FeedbackSensor.kAbsoluteEncoder));

        config.absoluteEncoder.apply(new AbsoluteEncoderConfig()
                .positionConversionFactor(kPositionConversionFactor)
                .velocityConversionFactor(kVelocityConversionFactor));

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        sim = new ArmSimulation(motor);
    }

    public void setSetpoint(double setpoint) {
        controller.setSetpoint(setpoint, ControlType.kPosition);
    }

    public void periodic() {}

    public void simulationPeriodic() {
        sim.simulationPeriodic();
    }
}
