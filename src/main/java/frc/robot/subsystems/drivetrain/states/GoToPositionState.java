package frc.robot.subsystems.drivetrain.states;

import static frc.robot.constants.AutoConstants.PathFollowConstants.*;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.TrajectoryBuilder;

public class GoToPositionState extends PositionalDriveState {
    private PathPlannerTrajectory trajectory;
    private Pose2d endPose;
    private Pose2d desiredPose;
    private Timer timer;

    public GoToPositionState(Drivetrain drivetrain, Pose2d pose) {
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
        timer.reset();
        timer.start();
        trajectory = TrajectoryBuilder.goToPosition(requiredSubsystem, endPose);
    }
    
    @Override
    public void execute() {
        State trajectoryState = trajectory.sample(timer.get());
        desiredPose = trajectoryState.getTargetHolonomicPose();
    
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        requiredSubsystem.drive(0, 0, 0);
    }
    
    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds() + 0.5;
    }
}
