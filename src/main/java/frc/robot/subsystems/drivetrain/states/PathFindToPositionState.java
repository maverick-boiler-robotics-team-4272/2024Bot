package frc.robot.subsystems.drivetrain.states;

import static frc.robot.constants.AutoConstants.PathFollowConstants.DEFAULT_POSE_DELTA;
import static frc.robot.constants.AutoConstants.PathFollowConstants.THETA_CONTROLLER;
import static frc.robot.constants.AutoConstants.PathFollowConstants.X_CONTROLLER;
import static frc.robot.constants.AutoConstants.PathFollowConstants.Y_CONTROLLER;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.paths.TrajectoryBuilder;

public class PathFindToPositionState extends PathFollowState {
    private Pose2d endPose;

    public PathFindToPositionState(
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

    public PathFindToPositionState(
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

    public PathFindToPositionState(
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

    public PathFindToPositionState(
        Drivetrain drivetrain,
        Pose2d pose,
        boolean positionStopped,
        boolean timeStopped,
        Pose2d positionDelta
    ) {
        this(drivetrain, pose, Y_CONTROLLER, X_CONTROLLER, THETA_CONTROLLER, positionStopped, timeStopped, positionDelta, false, false);
    }

    public PathFindToPositionState(
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

    public PathFindToPositionState(
        Drivetrain drivetrain,
        Pose2d pose,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        this(drivetrain, pose, X_CONTROLLER, Y_CONTROLLER, THETA_CONTROLLER, true, true, DEFAULT_POSE_DELTA, updateOdometry, updateFromAprilTag);
    }

    public PathFindToPositionState(
        Drivetrain drivetrain,
        Pose2d pose
    ) {
        this(drivetrain, pose, X_CONTROLLER, Y_CONTROLLER, THETA_CONTROLLER, true, true, DEFAULT_POSE_DELTA, false, false);
    }
    
    @Override
    public void initialize() {
        // setPath(TrajectoryBuilder.pathFindToPosition(requiredSubsystem, endPose));

        // TODO: Fix this. Make it work with the new pathing.
        setPath(null);
        super.initialize();
    }
}
