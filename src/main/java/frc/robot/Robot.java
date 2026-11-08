// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import java.util.List;

public class Robot extends TimedRobot {
    private final XboxController controller = new XboxController(0);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
    // to 1.
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    private final SimulatedDrivetrain drive = new SimulatedDrivetrain(false);
    private final LTVUnicycleController feedback = new LTVUnicycleController(0.020);
    private final Timer timer = new Timer();
    private final Trajectory trajectory;

    private Pose2d initialPose = new Pose2d(2, 2, Rotation2d.kZero);

    /** Called once at the beginning of the robot program. */
    public Robot() {
        // A very simple path that the robot will follow in the autonomous period.
        trajectory = TrajectoryGenerator.generateTrajectory(
                initialPose,
                List.of(new Translation2d(-1, -1)),
                new Pose2d(6, 4, Rotation2d.kZero),
                new TrajectoryConfig(2, 2));

        drive.resetOdometry(initialPose);
    }

    @Override
    public void robotPeriodic() {
        drive.periodic();
    }

    @Override
    public void autonomousInit() {
        timer.restart();
        drive.resetOdometry(initialPose);
    }

    @Override
    public void autonomousPeriodic() {
        // The trajectory planner calculates the position and speed at which the robot
        // should be moving based on the current time.
        // Then, the feedback controller determines the speed the robot should move at,
        // given its current position, to best match the trajectory.
        double elapsed = timer.get();
        Trajectory.State reference = trajectory.sample(elapsed);
        ChassisSpeeds speeds = feedback.calculate(drive.getPose(), reference);
        drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    @Override
    public void teleopPeriodic() {
        double forwardControl = controller.getLeftY();
        double turnControl = controller.getRightX();

        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        // Apply cosine compensation, which allows the robot to turn more easily
        // at high speeds.
        double xSpeed = -speedLimiter.calculate(forwardControl * Math.cos(turnControl * 2 / Math.PI))
                * Constants.Drivetrain.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        double rot = -rotLimiter.calculate(controller.getRightX()) * Constants.Drivetrain.kMaxAngularSpeed;
        drive.drive(xSpeed, rot);
    }

    @Override
    public void simulationPeriodic() {
        drive.simulationPeriodic();
    }
}
