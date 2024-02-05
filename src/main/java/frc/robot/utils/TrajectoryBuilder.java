package frc.robot.utils;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TrajectoryBuilder {
    public static PathPlannerPath buildPath(Pose2d start, Pose2d end) {
        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.bezierFromPoses(start, end), 
            new PathConstraints(
                3, 
                3, 
                Rotation2d.fromDegrees(540).getRotations(), 
                Rotation2d.fromDegrees(720).getRotations()
            ), 
            new GoalEndState(0, end.getRotation())
        );

        return path;
    }

    public static PathPlannerTrajectory goToPosition(Drivetrain drivetrain, Pose2d position) {
        PathPlannerTrajectory trajectory = new PathPlannerTrajectory(
            buildPath(drivetrain.getRobotPose(), position),
            drivetrain.getChassisSpeeds(), 
            drivetrain.getRobotPose().getRotation()
        );

        return trajectory;
    }
}
