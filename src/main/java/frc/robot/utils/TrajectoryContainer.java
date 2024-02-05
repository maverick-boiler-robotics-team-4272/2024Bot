package frc.robot.utils;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TrajectoryContainer {
    private static final ChassisSpeeds INITIAL_SPEEDS = new ChassisSpeeds(0, 0, 0);
    private static final Rotation2d INITIAL_ROTATION = new Rotation2d(0);

    public final PathPlannerTrajectory TEST_PATH;
    public final PathPlannerTrajectory TUNE_PATH;
    public final PathPlannerTrajectory STRESS_TEST_PATH;
    public final PathPlannerTrajectory HOMEMADE_PATH = new PathPlannerTrajectory(
        new PathPlannerPath(
            PathPlannerPath.bezierFromPoses(
                new Pose2d(1.5, 2, new Rotation2d(0)), 
                new Pose2d(1.5, 5, new Rotation2d(Math.PI / 2))), 
            new PathConstraints(2, 3, 0.5, 2), 
            new GoalEndState(0, new Rotation2d(Math.PI / 2))), 
        INITIAL_SPEEDS, 
        INITIAL_ROTATION
    );

    public TrajectoryContainer(String prefix) {
        TEST_PATH = PathPlannerPath.fromPathFile(prefix + " Test Path").getTrajectory(INITIAL_SPEEDS, INITIAL_ROTATION);
        TUNE_PATH = PathPlannerPath.fromPathFile(prefix + " Tune Path").getTrajectory(INITIAL_SPEEDS, INITIAL_ROTATION);
        STRESS_TEST_PATH = PathPlannerPath.fromPathFile(prefix + " Stress Test Path").getTrajectory(INITIAL_SPEEDS, Rotation2d.fromDegrees(-90));

    }
}
