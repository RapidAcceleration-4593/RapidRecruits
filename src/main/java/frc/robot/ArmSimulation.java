package frc.robot;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmSimulation {
    public static final Parameters kDefaultParams =
            new Parameters(DCMotor.getNEO(2), 1.2, 10, 1, 0, Math.toRadians(90), true, 0);

    public static record Parameters(
            DCMotor gearbox,
            double moiJKgMetersSquared,
            double gearing,
            double armLengthMeters,
            double minAngleRads,
            double maxAngleRads,
            boolean simulateGravity,
            double startingAngleRads) {}

    private final SingleJointedArmSim armSim;
    private final Parameters params;

    private final Mechanism2d armDisplay;
    private final MechanismRoot2d armRoot;
    private final MechanismLigament2d armMech;

    private final SparkMaxSim sparkSim;
    private final EncoderSim encoderSim;

    public ArmSimulation(Encoder encoder, SparkMax sparkMax) {
        this(encoder, sparkMax, kDefaultParams);
    }

    public ArmSimulation(Encoder encoder, SparkMax sparkMax, Parameters params) {
        this.params = params;
        LinearSystem<N2, N1, N2> armPlant =
                LinearSystemId.createSingleJointedArmSystem(params.gearbox, params.moiJKgMetersSquared, params.gearing);

        armSim = new SingleJointedArmSim(
                armPlant,
                params.gearbox,
                params.gearing,
                params.armLengthMeters,
                params.minAngleRads,
                params.maxAngleRads,
                params.simulateGravity,
                params.startingAngleRads);

        armDisplay = new Mechanism2d(100, 100, new Color8Bit(0, 0, 0));
        armRoot = armDisplay.getRoot("ArmPivot", 20, 20);
        armMech = new MechanismLigament2d("Arm", 70, params.minAngleRads, 10, new Color8Bit(255, 255, 255));
        armRoot.append(armMech);
        sparkSim = new SparkMaxSim(sparkMax, params.gearbox);
        encoderSim = new EncoderSim(encoder);
    }

    public void simulationPeriodic() {
        double railVoltage = RoboRioSim.getVInVoltage();
        double velocityRPM = armSim.getVelocityRadPerSec() * params.gearing * 60 / 2 / Math.PI;
        sparkSim.iterate(velocityRPM, railVoltage, 0.02);
        armSim.setInputVoltage(sparkSim.getAppliedOutput() * railVoltage);
        armSim.update(0.02);
        encoderSim.setDistance(armSim.getAngleRads());
        armMech.setAngle(new Rotation2d(armSim.getAngleRads()));
        SmartDashboard.putData("ArmMechanism", armDisplay);
    }
}
