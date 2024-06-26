package frc.robot.subsystems.drivetrain.states;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.misc.Pausable;
import frc.robot.utils.paths.Path;
import frc.robot.subsystems.drivetrain.drivers.PathDrivers;

import static frc.robot.constants.AutoConstants.PathFollowConstants.*;
import static frc.robot.constants.TelemetryConstants.Limelights.*;

public class PathFollowState extends AbstractDriveState<PathDrivers.XDriver, PathDrivers.YDriver, PathDrivers.ThetaDriver> implements Pausable {
    private Path path;
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
        Path path,
        PIDController xController,
        PIDController yController,
        PIDController thetaController,
        boolean positionStopped,
        boolean timeStopped,
        Pose2d positionDelta,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        super(
            drivetrain,
            new PathDrivers.XDriver(drivetrain, xController),
            new PathDrivers.YDriver(drivetrain, yController),
            new PathDrivers.ThetaDriver(drivetrain, thetaController)
        );
        
        this.path = path;
        this.trajectory = path == null ? null : path.trajectory;
        this.timer = new Timer();
        this.positionStopped = positionStopped;
        this.timeStopped = timeStopped;
        this.positionDelta = positionDelta;
        this.updateOdometry = updateOdometry;
        this.updateFromAprilTag = updateFromAprilTag;
    }

    public PathFollowState(
        Drivetrain drivetrain,
        Path path,
        boolean positionStopped,
        boolean timeStopped,
        Pose2d positionDelta,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        super(
            drivetrain,
            new PathDrivers.XDriver(drivetrain),
            new PathDrivers.YDriver(drivetrain),
            new PathDrivers.ThetaDriver(drivetrain)
        );

        this.path = path;
        this.trajectory = path == null ? null : path.trajectory;
        this.timer = new Timer();
        this.positionStopped = positionStopped;
        this.timeStopped = timeStopped;
        this.positionDelta = positionDelta;
        this.updateOdometry = updateOdometry;
        this.updateFromAprilTag = updateFromAprilTag;
    }

    public PathFollowState(
        Drivetrain drivetrain,
        Path path,
        PIDController xController,
        PIDController yController,
        PIDController thetaController,
        boolean positionStopped,
        boolean timeStopped,
        Pose2d positionDelta
    ) {
        this(drivetrain, path, xController, yController, thetaController, positionStopped, timeStopped, positionDelta, false, false);
    }

    public PathFollowState(
        Drivetrain drivetrain,
        Path path,
        boolean positionStopped,
        boolean timeStopped,
        Pose2d positionDelta
    ) {
        this(drivetrain, path, positionStopped, timeStopped, positionDelta, false, false);
    }

    public PathFollowState(
        Drivetrain drivetrain,
        Path path,
        PIDController xController,
        PIDController yController,
        PIDController thetaController,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        this(drivetrain, path, xController, yController, thetaController, true, true, DEFAULT_POSE_DELTA, updateOdometry, updateFromAprilTag);
    }

    public PathFollowState(
        Drivetrain drivetrain,
        Path path,
        boolean updateOdometry,
        boolean updateFromAprilTag
    ) {
        this(drivetrain, path, true, true, DEFAULT_POSE_DELTA, updateOdometry, updateFromAprilTag);
    }

    public PathFollowState(
        Drivetrain drivetrain,
        Path path
    ) {
        this(drivetrain, path, true, true, DEFAULT_POSE_DELTA, false, false);
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

    protected void setPath(Path path) {
        this.path = path;
        this.trajectory = path == null ? null : path.trajectory;
    }

    @Override
    public boolean isFieldRelative() {
        return true;
    }

    @Override
    public void initialize() {
        if(path == null)
            return;
        super.initialize();

        endPose = path.trajectory.getEndState().getTargetHolonomicPose();
        timer.restart();

        if(updateOdometry) {
            if(updateFromAprilTag && FRONT_LIMELIGHT.isValidTarget()) {
                requiredSubsystem.setRobotPose(FRONT_LIMELIGHT.getRobotPose());
                requiredSubsystem.setGyroscopeReading(FRONT_LIMELIGHT.getRobotPose().getRotation());
            } else {
                Pose2d initHolo = trajectory.getInitialTargetHolonomicPose();
                Pose2d initialPose = new Pose2d(initHolo.getTranslation(), path.initialPathRotation);

                //TODO: figure out why robo no roto
                requiredSubsystem.setGyroscopeReading(path.initialPathRotation);
                requiredSubsystem.setRobotPose(initialPose);
            }
        }
    }

    @Override
    public void execute() {
        if(path == null)
            return;
        
        desiredState = trajectory.sample(timer.get());
        desiredPose = desiredState.getTargetHolonomicPose();

        xDriver.setDesiredXPosition(desiredPose.getX());
        yDriver.setDesiredYPosition(desiredPose.getY());
        thetaDriver.setDesiredAngle(desiredPose.getRotation());

        xDriver.setFeedForward(desiredState.velocityMps * desiredState.heading.getCos());
        yDriver.setFeedForward(desiredState.velocityMps * desiredState.heading.getSin());

        if(desiredState.holonomicAngularVelocityRps.isPresent()) {
            thetaDriver.setFeedForward(desiredState.holonomicAngularVelocityRps.get());
        } else {
            thetaDriver.setFeedForward(0.0);
        }

        drive();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        if(path == null)
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

    public void pause() {
        timer.stop();
    }

    public void unpause() {
        timer.start();
    }

    public static boolean posesAlmostEqual(Pose2d a, Pose2d b, Pose2d delta) {
        return Math.abs(a.getX() - b.getX()) < delta.getX() &&
               Math.abs(a.getY() - b.getY()) < delta.getY() &&
               Math.abs(a.getRotation().getRadians() - b.getRotation().getRadians()) < delta.getRotation().getRadians();
    }
}
