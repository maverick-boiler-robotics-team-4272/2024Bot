package frc.robot.subsystems.drivetrain.states;

import static frc.robot.constants.AutoConstants.PathFollowConstants.*;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.TrajectoryBuilder;

public class GoToPositionState extends PathFollowState {
    private Pose2d endPose;

    public GoToPositionState(
        Drivetrain drivetrain,
        Pose2d pose,
        PIDController xController,
        PIDController yController,
        PIDController thetaController,
        boolean timeStopped,
        boolean positionStopped,
        Pose2d poseDelta,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        super(drivetrain, null, xController, yController, thetaController, timeStopped, positionStopped, poseDelta, updateOdometry, updateFromAprilTag);
        this.endPose = pose;
    }

    public GoToPositionState(
        Drivetrain drivetrain,
        Pose2d pose,
        boolean positionStopped,
        boolean timeStopped,
        Pose2d positionDelta,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        this(drivetrain, pose, X_CONTROLLER, Y_CONTROLLER, THETA_CONTROLLER, positionStopped, timeStopped, positionDelta, updateOdometry, updateFromAprilTag);
    }

    public GoToPositionState(
        Drivetrain drivetrain,
        Pose2d pose,
        PIDController xController,
        PIDController yController,
        PIDController thetaController,
        boolean positionStopped,
        boolean timeStopped,
        Pose2d positionDelta
    ) {
        this(drivetrain, pose, xController, yController, thetaController, positionStopped, timeStopped, positionDelta, false, false);
    }

    public GoToPositionState(
        Drivetrain drivetrain,
        Pose2d pose,
        boolean positionStopped,
        boolean timeStopped,
        Pose2d positionDelta
    ) {
        this(drivetrain, pose, Y_CONTROLLER, X_CONTROLLER, THETA_CONTROLLER, positionStopped, timeStopped, positionDelta, false, false);
    }

    public GoToPositionState(
        Drivetrain drivetrain,
        Pose2d pose,
        PIDController xController,
        PIDController yController,
        PIDController thetaController,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        this(drivetrain, pose, xController, yController, thetaController, true, true, DEFAULT_POSE_DELTA, updateOdometry, updateFromAprilTag);
    }

    public GoToPositionState(
        Drivetrain drivetrain,
        Pose2d pose,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        this(drivetrain, pose, X_CONTROLLER, Y_CONTROLLER, THETA_CONTROLLER, true, true, DEFAULT_POSE_DELTA, updateOdometry, updateFromAprilTag);
    }

    public GoToPositionState(
        Drivetrain drivetrain,
        Pose2d pose
    ) {
        this(drivetrain, pose, X_CONTROLLER, Y_CONTROLLER, THETA_CONTROLLER, true, true, DEFAULT_POSE_DELTA, false, false);
    }
    
    @Override
    public void initialize() {
        trajectory = TrajectoryBuilder.goToPosition(requiredSubsystem, endPose);

        super.initialize();
    }
}
