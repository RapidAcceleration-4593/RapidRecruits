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
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem {
    public static final double kEncoderToArmGearing = 1;
    public final ArmSimulation.Parameters params = ArmSimulation.kDefaultParams;

    private final ArmSimulation sim;
    private final SparkMax motor = new SparkMax(0, MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();
    private double kS = 0;
    private double kV = 0; // 0.0192;
    private double kA = 0.00;
    private double kCos = 0; // 2.00;

    private double kPositionConversionFactor =
            2 * Math.PI * kEncoderToArmGearing; // Converts encoder rotations to arm radians
    private double kVelocityConversionFactor =
            2 * Math.PI / 60 * kEncoderToArmGearing; // Converts encoder RPM to arm rad/s

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(3, 2);
    private final ProfiledPIDController pid = new ProfiledPIDController(1.0, 0, 0.4, constraints);

    private final ArmFeedforward feedforward = new ArmFeedforward(0, 4, 0.2, 0.5, 0.02);

    TrapezoidProfile.State setpointState = new State(0, 0);
    TrapezoidProfile.State goalState = new State(0, 0);
    Timer profileTimer = new Timer();

    public ArmSubsystem() {
        FeedForwardConfig ffConfig =
                new FeedForwardConfig().svacr(kS, kV, kA, kCos, 1); // 1 / kPositionConversionFactor);
        ClosedLoopConfig clConfig = new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(3.0, 0.0, 0.2);
        MAXMotionConfig mmConfig =
                new MAXMotionConfig().maxAcceleration(40).cruiseVelocity(100).allowedProfileError(10.0);
        clConfig.apply(ffConfig);
        clConfig.apply(mmConfig);
        AbsoluteEncoderConfig aeConfig = new AbsoluteEncoderConfig()
                .positionConversionFactor(kPositionConversionFactor)
                .velocityConversionFactor(kVelocityConversionFactor);

        SparkMaxConfig config = new SparkMaxConfig();
        config.apply(clConfig);
        config.apply(aeConfig);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        sim = new ArmSimulation(motor, kEncoderToArmGearing, params);
    }

    public void setSetpoint(double setpoint) {
        this.goalState = new State(setpoint, 0);
    }

    public void setVoltage(double voltage) {
        controller.setSetpoint(voltage, ControlType.kVoltage);
    }

    public void periodic() {
        double pos = motor.getAbsoluteEncoder().getPosition();
        double vel = motor.getAbsoluteEncoder().getVelocity();

        double fb = pid.calculate(pos, goalState);
        double ff = 4.0 * Math.cos(pid.getSetpoint().position);

        motor.setVoltage(fb + ff);

        SmartDashboard.putNumber("FF", ff);
        SmartDashboard.putNumber("FB", fb);

        SmartDashboard.putNumber(
                "ArmVelocityRadPerSec", motor.getAbsoluteEncoder().getVelocity());
        SmartDashboard.putNumber("Setpoint", goalState.position);
        SmartDashboard.putNumber("Profile", pid.getSetpoint().position);
    }

    public void simulationPeriodic() {
        sim.simulationPeriodic();
    }
}
