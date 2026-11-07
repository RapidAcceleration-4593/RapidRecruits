// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Drivetrain.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;

/** A simple differential drivetrain that can be extended with simulation capabilities. */
public class Drivetrain {
    // Protected modifiers allow subclasses to access fields
    protected final SparkMax leftLeader = new SparkMax(leftLeaderId, MotorType.kBrushless);
    protected final SparkMax leftFollower = new SparkMax(leftFollowerId, MotorType.kBrushless);
    protected final SparkMax rightLeader = new SparkMax(rightLeaderId, MotorType.kBrushless);
    protected final SparkMax rightFollower = new SparkMax(rightFollowerId, MotorType.kBrushless);

    private final Encoder leftEncoder = new Encoder(leftEncoderAId, leftEncoderBId);
    private final Encoder rightEncoder = new Encoder(rightEncoderAId, rightEncoderBId);

    private final PIDController leftPIDController = new PIDController(kP, kI, kD);
    private final PIDController rightPIDController = new PIDController(kP, kI, kD);

    private final AnalogGyro gyro = new AnalogGyro(gyroId);

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);
    private final DifferentialDriveOdometry odometry =
            new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    // Gains are for example purposes only - must be determined for your own
    // robot!
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

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

    /** Sets speeds to the drivetrain motors. */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
        double leftOutput = leftPIDController.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
        double rightOutput = rightPIDController.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);

        leftLeader.setVoltage(leftOutput + leftFeedforward);
        rightLeader.setVoltage(rightOutput + rightFeedforward);
    }

    /**
     * Controls the robot using arcade drive.
     *
     * @param xSpeed the speed for the x axis
     * @param rot the rotation
     */
    public void drive(double xSpeed, double rot) {
        setSpeeds(kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
    }

    /** Update robot odometry. */
    public void updateOdometry() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }

    /** Resets robot odometry. */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
    }

    /** Check the current robot pose. */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /** Update odometry - this should be run every robot loop. */
    public void periodic() {
        updateOdometry();
    }
}
