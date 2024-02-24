package frc.robot.subsystems.drivetrain.states;

import static frc.robot.constants.AutoConstants.PathFollowConstants.DEFAULT_POSE_DELTA;
import static frc.robot.constants.AutoConstants.PathFollowConstants.THETA_CONTROLLER;
import static frc.robot.constants.AutoConstants.PathFollowConstants.X_CONTROLLER;
import static frc.robot.constants.AutoConstants.PathFollowConstants.Y_CONTROLLER;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.paths.TrajectoryBuilder;

public class PathFindToPositionState extends PositionalDriveState {
    private PathPlannerTrajectory trajectory;
    private Pose2d endPose;
    private Pose2d desiredPose;
    private Timer timer;

    public PathFindToPositionState(Drivetrain drivetrain, Pose2d pose) {
        super(drivetrain, X_CONTROLLER, Y_CONTROLLER, THETA_CONTROLLER);
        this.endPose = pose;
        this.timer = new Timer();
    }

    @Override
    public double getDesiredX() {
        return desiredPose.getX();
    }

    @Override
    public double getDesiredY() {
        return desiredPose.getY();
    }

    @Override
    public Rotation2d getDesiredTheta() {
        return desiredPose.getRotation();
    }
    
    @Override
    public void initialize() {
        super.initialize();

        timer.reset();
        timer.start();
        trajectory = TrajectoryBuilder.pathFindToPosition(requiredSubsystem, endPose);
    }
    
    @Override
    public void execute() {
        if(trajectory == null)
            return;
        State trajectoryState = trajectory.sample(timer.get());
        desiredPose = trajectoryState.getTargetHolonomicPose();
    
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        timer.stop();
    }
    
    @Override
    public boolean isFinished() {
        if(trajectory == null)
            return true;
        return PathFollowState.posesAlmostEqual(requiredSubsystem.getRobotPose(), endPose, DEFAULT_POSE_DELTA) && timer.get() >= trajectory.getTotalTimeSeconds() + 1.0;
    }
}
