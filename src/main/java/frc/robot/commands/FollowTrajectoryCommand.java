package frc.robot.commands;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectoryCommand extends ParallelRaceGroup {

    private Drivetrain drive;
    private Trajectory trajectory;
    private Timer timer = new Timer();
    private LTVUnicycleController feedback = new LTVUnicycleController(0.020);

    private double driveSpeed = 0;
    private double turnSpeed = 0;

    public FollowTrajectoryCommand(Drivetrain drive, Trajectory trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;
        addCommands(calculateTrajectory(), drive.drive(this::getDriveSpeed, this::getTurnSpeed));
    }

    public void calcInit() {
        timer.restart();
    }

    private void calcExec() {
        double elapsed = timer.get();
        Trajectory.State reference = trajectory.sample(elapsed);
        ChassisSpeeds speeds = feedback.calculate(drive.getPose(), reference);
        driveSpeed = speeds.vxMetersPerSecond;
        turnSpeed = speeds.omegaRadiansPerSecond;
    }

    private boolean calcIsFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds();
    }

    private Command calculateTrajectory() {
        return Commands.runOnce(this::calcInit).andThen(Commands.run(this::calcExec)).until(this::calcIsFinished);
    }

    private double getDriveSpeed() {
        return driveSpeed;
    }

    private double getTurnSpeed() {
        return turnSpeed;
    }
}
