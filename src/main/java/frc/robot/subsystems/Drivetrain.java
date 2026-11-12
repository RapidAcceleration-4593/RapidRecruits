// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** A simple differential drivetrain that can be extended with simulation capabilities. */
public class Drivetrain extends SubsystemBase {
    // Protected modifiers allow subclasses to access fields.
    // We do this to allow the simulation class to modify the motors.
    // SparkMaxes are motor controllers.
    protected final SparkMax leftLeader = new SparkMax(leftLeaderId, MotorType.kBrushless);
    protected final SparkMax leftFollower = new SparkMax(leftFollowerId, MotorType.kBrushless);
    protected final SparkMax rightLeader = new SparkMax(rightLeaderId, MotorType.kBrushless);
    protected final SparkMax rightFollower = new SparkMax(rightFollowerId, MotorType.kBrushless);

    // Encoders measure the rotation of the wheels, AKA how far they have driven.
    private final Encoder leftEncoder = new Encoder(leftEncoderAId, leftEncoderBId);
    private final Encoder rightEncoder = new Encoder(rightEncoderAId, rightEncoderBId);

    // A gyro measures the robot's orientation on the field.
    private final AnalogGyro gyro = new AnalogGyro(gyroId);

    // PID controllers allow for feedback control: changing how fast we drive the motor
    // depending on how close we are to the setpoint.
    private final PIDController leftPIDController = new PIDController(kP, kI, kD);
    private final PIDController rightPIDController = new PIDController(kP, kI, kD);

    // Motor feedforwards predict how much voltage will be needed to reach a certain motor velocity.
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

    // The kinematics describe the physical properties of the robot.
    // They help us predict how it will move.
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);

    // Odometry lets us estimate where the robot is on the field, based off of the gyro
    // and encoder readings. This is highly useful for the autonomous period.
    private final DifferentialDriveOdometry odometry =
            new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    // A publisher allows us to send data over Network Tables to AdvantageScope.
    private final StructPublisher<Pose2d> estimatedPosePub = NetworkTableInstance.getDefault()
            .getStructTopic("EstimatedPose", Pose2d.struct)
            .publish();

    /** Subsystem constructor. */
    public Drivetrain() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        SparkBaseConfig rightLeaderConfig = new SparkMaxConfig().inverted(true);

        // There are four motors total: two leaders and two followers.
        // The following motors simply match the speed of the leaders.
        SparkBaseConfig leftFollowConfig = new SparkMaxConfig().follow(leftLeader);
        SparkBaseConfig rightFollowConfig = new SparkMaxConfig().follow(rightLeader);

        // The ResetMode determines whether all the other configuration options are reset to default values
        // The PersistMode determines if the configuration should be written to storage and persist
        // through power cycling.
        rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftFollower.configure(leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFollower.configure(rightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

        leftEncoder.reset();
        rightEncoder.reset();
    }

    /**
     * Sets speeds to the drivetrain motors using arcade drive format.
     *
     * @param xSpeed the speed for the x axis
     * @param rot the rotation
     */
    private void setSpeeds(double xSpeed, double rot) {
        setSpeeds(kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
    }

    /** Sets speeds to the drivetrain motors. */
    private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        // Use feedforward and feedback mechanisms to calculate how much voltage we need
        // to reach the setpoint.
        final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
        double leftOutput = leftPIDController.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
        double rightOutput = rightPIDController.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);

        leftLeader.setVoltage(leftOutput + leftFeedforward);
        rightLeader.setVoltage(rightOutput + rightFeedforward);
    }

    /** Update robot odometry. */
    private void updateOdometry() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }

    public Command drive(DoubleSupplier driveSupplier, DoubleSupplier turnSupplier) {
        return run(() -> setSpeeds(driveSupplier.getAsDouble(), turnSupplier.getAsDouble()));
    }

    public Command drive(Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds) {
        return run(() -> setSpeeds(wheelSpeeds.get()));
    }

    /** Resets robot odometry. */
    public Command resetOdometry(Pose2d pose) {
        return new InstantCommand(
                () -> {
                    leftEncoder.reset();
                    rightEncoder.reset();
                    gyro.reset();
                    odometry.resetPosition(Rotation2d.kZero, 0, 0, pose);
                },
                this);
    }

    /** Check the current robot pose. */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /** Update odometry - this should be run every robot loop. */
    public void periodic() {
        updateOdometry();
        estimatedPosePub.set(odometry.getPoseMeters());
    }
}
