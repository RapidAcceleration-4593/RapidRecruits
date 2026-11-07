package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

/**
 * A {@link Drivetrain} with simulation support. Understanding simulation is not important for this activity.
 *
 * @see Drivetrain
 */
public class SimulatedDrivetrain extends Drivetrain {
    // Simulation classes help us simulate our robot
    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(gyro);
    private final EncoderSim m_leftEncoderSim = new EncoderSim(leftEncoder);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(rightEncoder);
    private final StructPublisher<Pose2d> simPosePub = NetworkTableInstance.getDefault()
            .getStructTopic("SimulationPose", Pose2d.struct)
            .publish();
    private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
            LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
    private final DifferentialDrivetrainSim m_drivetrainSimulator =
            new DifferentialDrivetrainSim(m_drivetrainSystem, DCMotor.getCIM(2), 8, kTrackWidth, kWheelRadius, null);

    /** Resets robot odometry. */
    public void resetOdometry(Pose2d pose) {
        super.resetOdometry(pose);
        m_drivetrainSimulator.setPose(pose);
    }

    /** Update our simulation. This should be run every robot loop in simulation. */
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.
        m_drivetrainSimulator.setInputs(
                leftLeader.get() * RobotController.getInputVoltage(),
                rightLeader.get() * RobotController.getInputVoltage());
        m_drivetrainSimulator.update(0.02);

        m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    }

    /** Update odometry - this should be run every robot loop. */
    public void periodic() {
        super.periodic();
        simPosePub.set(m_drivetrainSimulator.getPose());
    }
}
