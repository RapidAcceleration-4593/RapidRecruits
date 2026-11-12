package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.List;

public class Autos {

    public static Command basicAuto(DrivetrainSubsystem drivetrain) {
        Pose2d startPose = new Pose2d(2, 2, Rotation2d.kZero);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                startPose,
                List.of(new Translation2d(-1, -1)),
                new Pose2d(6, 4, Rotation2d.kZero),
                new TrajectoryConfig(2, 2));
        return drivetrain.resetOdometry(startPose).andThen(new FollowTrajectoryCommand(drivetrain, trajectory));
    }
}
