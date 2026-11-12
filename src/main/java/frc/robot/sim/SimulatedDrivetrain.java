package frc.robot.sim;

import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.DrivetrainConstants.gyroId;
import static frc.robot.Constants.DrivetrainConstants.leftEncoderAId;
import static frc.robot.Constants.DrivetrainConstants.rightEncoderAId;

import com.revrobotics.sim.SparkMaxSim;
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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * A {@link Drivetrain} with simulation support. Understanding simulation is not important for this activity.
 *
 * @see Drivetrain
 */
public class SimulatedDrivetrain extends Drivetrain {
    // Simulation classes help us simulate our robot
    private final AnalogGyroSim gyroSim = new AnalogGyroSim(gyroId);
    private final EncoderSim leftEncoderSim = EncoderSim.createForChannel(leftEncoderAId);
    private final EncoderSim rightEncoderSim = EncoderSim.createForChannel(rightEncoderAId);
    private final SparkMaxSim leftLeaderSim = new SparkMaxSim(leftLeader, DCMotor.getNEO(2));
    private final SparkMaxSim rightLeaderSim = new SparkMaxSim(rightLeader, DCMotor.getNEO(2));

    private final LinearSystem<N2, N2, N2> drivetrainSystem =
            LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
    private final DifferentialDrivetrainSim drivetrainSimulator = new DifferentialDrivetrainSim(
            drivetrainSystem, DCMotor.getNEO(2), kDriveGearing, kTrackWidth, kWheelRadius, null);

    private final StructPublisher<Pose2d> simPosePub = NetworkTableInstance.getDefault()
            .getStructTopic("SimulationPose", Pose2d.struct)
            .publish();

    /** Resets robot odometry. */
    public Command resetOdometry(Pose2d pose) {
        return super.resetOdometry(pose).andThen(() -> drivetrainSimulator.setPose(pose));
    }

    /** Update our simulation. This should be run every robot loop in simulation. */
    public void simulationPeriodic() {

        double leftInversionFactor = leftLeader.configAccessor.getInverted() ? -1 : 1;
        double rightInversionFactor = rightLeader.configAccessor.getInverted() ? 1 : -1;

        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.

        double leftWheelRotationsRPM =
                calculateMotorRPM(drivetrainSimulator.getLeftVelocityMetersPerSecond()) * leftInversionFactor;
        double rightWheelRotationsRPM =
                calculateMotorRPM(drivetrainSimulator.getRightVelocityMetersPerSecond()) * rightInversionFactor;

        leftLeaderSim.iterate(leftWheelRotationsRPM, RobotController.getBatteryVoltage(), 0.02);
        rightLeaderSim.iterate(rightWheelRotationsRPM, RobotController.getBatteryVoltage(), 0.02);

        drivetrainSimulator.setInputs(
                leftLeaderSim.getAppliedOutput() * RobotController.getBatteryVoltage() * leftInversionFactor,
                rightLeaderSim.getAppliedOutput() * RobotController.getBatteryVoltage() * rightInversionFactor);
        drivetrainSimulator.update(0.02);

        leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters() * leftInversionFactor);
        leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond() * leftInversionFactor);
        rightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters() * rightInversionFactor);
        rightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond() * rightInversionFactor);
        gyroSim.setAngle(-drivetrainSimulator.getHeading().getDegrees());
    }

    /** Update odometry - this should be run every robot loop. */
    public void periodic() {
        super.periodic();
        simPosePub.set(drivetrainSimulator.getPose());
    }

    private double calculateMotorRPM(double wheelTangentalVelocity) {
        // Convert tangental velocity (m/s) to rad/s to rot/s to RPM
        return wheelTangentalVelocity / kWheelRadius / 2 / Math.PI * 60 / kDriveGearing;
    }
}
