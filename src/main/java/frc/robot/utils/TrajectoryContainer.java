package frc.robot.utils;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TrajectoryContainer {
    private static final ChassisSpeeds INITIAL_SPEEDS = new ChassisSpeeds(0, 0, 0);
    private static final Rotation2d INITIAL_ROTATION = new Rotation2d(0);

    public final PathPlannerTrajectory TEST_PATH;
    public final PathPlannerTrajectory TUNE_PATH;

    public TrajectoryContainer(String prefix) {
        TEST_PATH = PathPlannerPath.fromPathFile(prefix + " Test Path").getTrajectory(INITIAL_SPEEDS, INITIAL_ROTATION);
        TUNE_PATH = PathPlannerPath.fromPathFile(prefix + " Tune Path").getTrajectory(INITIAL_SPEEDS, INITIAL_ROTATION);

    }
}
