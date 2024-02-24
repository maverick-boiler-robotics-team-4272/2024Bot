package frc.robot.subsystems.drivetrain.states;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.Drivetrain;

import static frc.robot.constants.AutoConstants.PathFollowConstants.*;
import static frc.robot.constants.TelemetryConstants.Limelights.*;

public class PathFollowState extends PositionalDriveState {
    private PathPlannerTrajectory trajectory;
    private Pose2d desiredPose;
    private State desiredState;
    private Pose2d endPose;

    private Timer timer;

    private boolean positionStopped;
    private boolean timeStopped;
    private Pose2d positionDelta;
    
    private boolean updateOdometry;
    private boolean updateFromAprilTag;

    public PathFollowState(
        Drivetrain drivetrain,
        PathPlannerTrajectory trajectory,
        PIDController xController,
        PIDController yController,
        PIDController thetaController,
        boolean positionStopped,
        boolean timeStopped,
        Pose2d positionDelta,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        super(drivetrain, xController, yController, thetaController);
        
        this.trajectory = trajectory;
        this.timer = new Timer();
        this.positionStopped = positionStopped;
        this.timeStopped = timeStopped;
        this.positionDelta = positionDelta;
        this.updateOdometry = updateOdometry;
        this.updateFromAprilTag = updateFromAprilTag;
    }

    public PathFollowState(
        Drivetrain drivetrain,
        PathPlannerTrajectory trajectory,
        boolean positionStopped,
        boolean timeStopped,
        Pose2d positionDelta,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        this(drivetrain, trajectory, X_CONTROLLER, Y_CONTROLLER, THETA_CONTROLLER, positionStopped, timeStopped, positionDelta, updateOdometry, updateFromAprilTag);
    }

    public PathFollowState(
        Drivetrain drivetrain,
        PathPlannerTrajectory trajectory,
        PIDController xController,
        PIDController yController,
        PIDController thetaController,
        boolean positionStopped,
        boolean timeStopped,
        Pose2d positionDelta
    ) {
        this(drivetrain, trajectory, xController, yController, thetaController, positionStopped, timeStopped, positionDelta, false, false);
    }

    public PathFollowState(
        Drivetrain drivetrain,
        PathPlannerTrajectory trajectory,
        boolean positionStopped,
        boolean timeStopped,
        Pose2d positionDelta
    ) {
        this(drivetrain, trajectory, Y_CONTROLLER, X_CONTROLLER, THETA_CONTROLLER, positionStopped, timeStopped, positionDelta, false, false);
    }

    public PathFollowState(
        Drivetrain drivetrain,
        PathPlannerTrajectory trajectory,
        PIDController xController,
        PIDController yController,
        PIDController thetaController,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        this(drivetrain, trajectory, xController, yController, thetaController, true, true, DEFAULT_POSE_DELTA, updateOdometry, updateFromAprilTag);
    }

    public PathFollowState(
        Drivetrain drivetrain,
        PathPlannerTrajectory trajectory,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        this(drivetrain, trajectory, X_CONTROLLER, Y_CONTROLLER, THETA_CONTROLLER, true, true, DEFAULT_POSE_DELTA, updateOdometry, updateFromAprilTag);
    }

    public PathFollowState(
        Drivetrain drivetrain,
        PathPlannerTrajectory trajectory
    ) {
        this(drivetrain, trajectory, X_CONTROLLER, Y_CONTROLLER, THETA_CONTROLLER, true, true, DEFAULT_POSE_DELTA, false, false);
    }

    public PathFollowState withEndConditions(boolean positionStopped, boolean timeStopped, Pose2d positionDelta) {
        this.positionStopped = positionStopped;
        this.timeStopped = timeStopped;
        this.positionDelta = positionDelta;

        return this;
    }

    public PathFollowState withInitializationConditions(boolean updateOdometry, boolean updateFromAprilTag) {
        this.updateOdometry = updateOdometry;
        this.updateFromAprilTag = updateFromAprilTag;

        return this;
    }

    protected void setTrajectory(PathPlannerTrajectory trajectory) {
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        if(trajectory == null)
            return;
        super.initialize();

        endPose = trajectory.getEndState().getTargetHolonomicPose();
        timer.restart();

        if(updateOdometry) {
            if(updateFromAprilTag && FRONT_LIMELIGHT.isValidTarget()) {
                requiredSubsystem.setRobotPose(FRONT_LIMELIGHT.getRobotPose());
                requiredSubsystem.setGyroscopeReading(FRONT_LIMELIGHT.getRobotPose().getRotation());
            } else {
                Pose2d initHolo = trajectory.getInitialTargetHolonomicPose();

                requiredSubsystem.setGyroscopeReading(initHolo.getRotation().unaryMinus());
                requiredSubsystem.setRobotPose(initHolo);
            }
        }
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
    public double getXFeedForward() {
        return desiredState.velocityMps * desiredState.heading.getCos();
    }

    @Override
    public double getYFeedForward() {
        return desiredState.velocityMps * desiredState.heading.getSin();
    }

    @Override
    public double getThetaFeedForward() {
        if(desiredState.holonomicAngularVelocityRps.isPresent())
            return desiredState.holonomicAngularVelocityRps.get();
        else
            return 0.0;
    }

    @Override
    public void execute() {
        desiredState = trajectory.sample(timer.get());
        desiredPose = desiredState.getTargetHolonomicPose();

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
        boolean time = false;
        boolean position = false;

        if(timeStopped) {
            time = timer.get() >= trajectory.getTotalTimeSeconds();
        }

        if(positionStopped) {
            position = posesAlmostEqual(requiredSubsystem.getRobotPose(), endPose, positionDelta);
        }

        return timeStopped && positionStopped ? time && position : time || position;
    }

    public static boolean posesAlmostEqual(Pose2d a, Pose2d b, Pose2d delta) {
        return Math.abs(a.getX() - b.getX()) < delta.getX() &&
               Math.abs(a.getY() - b.getY()) < delta.getY() &&
               Math.abs(a.getRotation().getRadians() - b.getRotation().getRadians()) < delta.getRotation().getRadians();
    }
}
