// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Drivetrain.*;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/** A simple differential drivetrain that can be extended with simulation capabilities. */
public class Drivetrain {

    // TODO: THINGS THAT NEED ADDED
    // Two follower motors
    // Two encoders
    // An AnalogGyro
    // A DifferentialDriveOdometry

    protected SparkMax leftLeader; // TODO: assign SparkMaxes values
    protected SparkMax rightLeader;

    // PID controllers allow for feedback control: changing how fast we drive the motor
    // depending on how close we are to the setpoint.
    private final PIDController leftPIDController = new PIDController(kP, kI, kD);
    private final PIDController rightPIDController = new PIDController(kP, kI, kD);

    // The kinematics describe the physical properties of the robot.
    // They help us predict how it will move.
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);

    // Motor feedforwards predict how much voltage will be needed to reach a certain motor velocity.
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

    // A publisher allows us to send data over Network Tables to AdvantageScope.
    private final StructPublisher<Pose2d> estimatedPosePub = NetworkTableInstance.getDefault()
            .getStructTopic("EstimatedPose", Pose2d.struct)
            .publish();

    /** Subsystem constructor. */
    public Drivetrain() {
        // TODO: Configure the two leader and two follower motors
    }

    /** Sets speeds to the drivetrain motors. */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        // Use feedforward and feedback mechanisms to calculate how much voltage we need
        // to reach the setpoint.
        final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
        // TODO: Implement feedback control
        double leftOutput = 0;//leftPIDController.calculate(<your code here>, speeds.leftMetersPerSecond);
        double rightOutput = 0;//rightPIDController.calculate(<your code here>, speeds.rightMetersPerSecond);

        // TODO: Set motor voltages
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
        // TODO: Update the odometry using the gyro yaw and encoder distance readings.
    }

    /** Resets robot odometry. */
    public void resetOdometry(Pose2d pose) {
        // TODO: Reset all sensors
        // Reset odometry using .resetPosition() method
    }

    /** Check the current robot pose. */
    public Pose2d getPose() {
        // TODO: Return odometry pose reading
        return Pose2d.kZero;
    }

    /** Update odometry - this should be run every robot loop. */
    public void periodic() {
        updateOdometry();
        estimatedPosePub.set(getPose());
    }
}
