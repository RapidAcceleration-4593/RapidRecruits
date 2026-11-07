// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/** A simple differential drivetrain that can be extended with simulation capabilities. */
public class Drivetrain {
    // 3 meters per second.
    public static final double kMaxSpeed = 3.0;
    // 1/2 rotation per second.
    public static final double kMaxAngularSpeed = Math.PI;

    // Protected modifiers allow subclasses to access fields
    protected static final double kTrackWidth = 0.381 * 2;
    protected static final double kWheelRadius = 0.0508;
    protected static final int kEncoderResolution = -4096;

    protected final PWMSparkMax leftLeader = new PWMSparkMax(1);
    protected final PWMSparkMax leftFollower = new PWMSparkMax(2);
    protected final PWMSparkMax rightLeader = new PWMSparkMax(3);
    protected final PWMSparkMax rightFollower = new PWMSparkMax(4);

    protected final Encoder leftEncoder = new Encoder(0, 1);
    protected final Encoder rightEncoder = new Encoder(2, 3);

    protected final PIDController leftPIDController = new PIDController(8.5, 0, 0);
    protected final PIDController rightPIDController = new PIDController(8.5, 0, 0);

    protected final AnalogGyro gyro = new AnalogGyro(0);

    protected final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);
    protected final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
            gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    // Gains are for example purposes only - must be determined for your own
    // robot!
    protected final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

    /** Subsystem constructor. */
    public Drivetrain() {
        leftLeader.addFollower(leftFollower);
        rightLeader.addFollower(rightFollower);

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        rightLeader.setInverted(true);

        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

        leftEncoder.reset();
        rightEncoder.reset();

        rightLeader.setInverted(true);
    }

    /** Sets speeds to the drivetrain motors. */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
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
        odometry.resetPosition(
                gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
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
