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

    public final PathPlannerTrajectory TWO_CENTER_RUSH;
    public final PathPlannerTrajectory THREE_PIECE_CLOSE;

    public TrajectoryContainer(String prefix) {
        TEST_PATH = PathPlannerPath.fromPathFile(prefix + " Test Path").getTrajectory(INITIAL_SPEEDS, INITIAL_ROTATION);
        TUNE_PATH = PathPlannerPath.fromPathFile(prefix + " Tune Path").getTrajectory(INITIAL_SPEEDS, INITIAL_ROTATION);
        STRESS_TEST_PATH = PathPlannerPath.fromPathFile(prefix + " Stress Test Path").getTrajectory(INITIAL_SPEEDS, Rotation2d.fromDegrees(-90));

        TWO_CENTER_RUSH = PathPlannerPath.fromPathFile(prefix + " Two Center Rush").getTrajectory(INITIAL_SPEEDS, Rotation2d.fromDegrees(180));
        THREE_PIECE_CLOSE = PathPlannerPath.fromPathFile(prefix + " Three Piece Close").getTrajectory(INITIAL_SPEEDS, Rotation2d.fromDegrees(180));
    }
}
