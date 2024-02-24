package frc.robot.utils.paths;

import static frc.robot.constants.RobotConstants.DrivetrainConstants.*;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TrajectoryBuilder {
    public static PathPlannerPath buildPath(Pose2d start, Pose2d end) {
        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.bezierFromPoses(new Pose2d(start.getTranslation(), start.minus(end).getRotation()), end), 
            new PathConstraints(
                MAX_TRANSLATIONAL_SPEED, 
                3.0, //3g's but sidways
                MAX_ROTATIONAL_SPEED, 
                Rotation2d.fromDegrees(720).getRadians()
            ), 
            new GoalEndState(0, end.getRotation())
        );

        return path;
    }

    public static PathPlannerTrajectory goToPosition(Drivetrain drivetrain, Pose2d position) {
        PathPlannerTrajectory trajectory = new PathPlannerTrajectory(
            buildPath(drivetrain.getRobotPose(), position),
            ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getChassisSpeeds(), drivetrain.getRobotPose().getRotation()), 
            drivetrain.getRobotPose().getRotation()
        );

        return trajectory;
    }

    public static PathPlannerTrajectory pathFindToPosition(Drivetrain drivetrain, Pose2d position) {
        Pathfinding.setStartPosition(drivetrain.getRobotPose().getTranslation());
        Pathfinding.setGoalPosition(position.getTranslation());

        PathPlannerPath foundPath = Pathfinding.getCurrentPath(new PathConstraints(
                MAX_TRANSLATIONAL_SPEED, 
                3.0, 
                MAX_ROTATIONAL_SPEED, 
                Rotation2d.fromDegrees(720).getRadians()
            ), 
            new GoalEndState(0, position.getRotation())
        );

        if(foundPath == null) {
            System.out.println("An error occurred in path generation");
            return null;
        }

        return new PathPlannerTrajectory(
            foundPath,
            ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getChassisSpeeds(), drivetrain.getRobotPose().getRotation()),
            drivetrain.getRobotPose().getRotation()
        );

    }

    static {
        Pathfinding.setPathfinder(new AStarPathfinding(0.1));
        Pathfinding.setDynamicObstacles(List.of(
            new Pair<Translation2d, Translation2d>(new Translation2d(5.90, 2.10), new Translation2d(6.00, 6.20)),
            new Pair<Translation2d, Translation2d>(new Translation2d(6.00, 2.10), new Translation2d(6.10, 6.10)),
            new Pair<Translation2d, Translation2d>(new Translation2d(5.30, 2.20), new Translation2d(5.90, 6.10)),
            new Pair<Translation2d, Translation2d>(new Translation2d(5.20, 2.30), new Translation2d(5.30, 6.00)),
            new Pair<Translation2d, Translation2d>(new Translation2d(4.90, 2.40), new Translation2d(5.20, 5.80)),
            new Pair<Translation2d, Translation2d>(new Translation2d(6.10, 2.40), new Translation2d(6.20, 5.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(10.70, 2.40), new Translation2d(11.30, 6.00)),
            new Pair<Translation2d, Translation2d>(new Translation2d(4.80, 2.50), new Translation2d(4.90, 5.70)),
            new Pair<Translation2d, Translation2d>(new Translation2d(10.60, 2.50), new Translation2d(10.70, 5.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(11.30, 2.50), new Translation2d(11.40, 6.00)),
            new Pair<Translation2d, Translation2d>(new Translation2d(11.40, 2.50), new Translation2d(11.50, 5.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(4.70, 2.60), new Translation2d(4.80, 5.70)),
            new Pair<Translation2d, Translation2d>(new Translation2d(10.50, 2.60), new Translation2d(10.60, 5.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(11.50, 2.60), new Translation2d(11.70, 5.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(11.70, 2.60), new Translation2d(11.80, 5.80)),
            new Pair<Translation2d, Translation2d>(new Translation2d(4.40, 2.70), new Translation2d(4.70, 5.50)),
            new Pair<Translation2d, Translation2d>(new Translation2d(6.20, 2.70), new Translation2d(6.30, 5.40)),
            new Pair<Translation2d, Translation2d>(new Translation2d(11.80, 2.70), new Translation2d(11.90, 5.80)),
            new Pair<Translation2d, Translation2d>(new Translation2d(11.90, 2.70), new Translation2d(12.00, 5.70)),
            new Pair<Translation2d, Translation2d>(new Translation2d(12.00, 2.70), new Translation2d(12.10, 5.60)),
            new Pair<Translation2d, Translation2d>(new Translation2d(4.20, 2.80), new Translation2d(4.40, 5.40)),
            new Pair<Translation2d, Translation2d>(new Translation2d(10.40, 2.80), new Translation2d(10.50, 5.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(12.10, 2.80), new Translation2d(12.20, 5.50)),
            new Pair<Translation2d, Translation2d>(new Translation2d(12.20, 2.80), new Translation2d(12.40, 5.40)),
            new Pair<Translation2d, Translation2d>(new Translation2d(12.40, 2.80), new Translation2d(12.50, 5.30)),
            new Pair<Translation2d, Translation2d>(new Translation2d(4.00, 2.90), new Translation2d(4.20, 5.30)),
            new Pair<Translation2d, Translation2d>(new Translation2d(12.50, 2.90), new Translation2d(12.70, 5.20)),
            new Pair<Translation2d, Translation2d>(new Translation2d(3.80, 3.00), new Translation2d(4.00, 5.20)),
            new Pair<Translation2d, Translation2d>(new Translation2d(12.70, 3.00), new Translation2d(12.80, 5.10)),
            new Pair<Translation2d, Translation2d>(new Translation2d(12.80, 3.00), new Translation2d(12.90, 5.00)),
            new Pair<Translation2d, Translation2d>(new Translation2d(3.70, 3.10), new Translation2d(3.80, 5.10)),
            new Pair<Translation2d, Translation2d>(new Translation2d(12.90, 3.10), new Translation2d(13.00, 5.00)),
            new Pair<Translation2d, Translation2d>(new Translation2d(13.00, 3.10), new Translation2d(13.10, 4.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(13.10, 3.10), new Translation2d(13.20, 4.80)),
            new Pair<Translation2d, Translation2d>(new Translation2d(3.60, 3.20), new Translation2d(3.70, 5.00)),
            new Pair<Translation2d, Translation2d>(new Translation2d(10.30, 3.20), new Translation2d(10.40, 5.70)),
            new Pair<Translation2d, Translation2d>(new Translation2d(13.20, 3.20), new Translation2d(13.30, 4.80)),
            new Pair<Translation2d, Translation2d>(new Translation2d(3.30, 3.30), new Translation2d(3.60, 4.80)),
            new Pair<Translation2d, Translation2d>(new Translation2d(13.30, 3.30), new Translation2d(13.40, 4.70)),
            new Pair<Translation2d, Translation2d>(new Translation2d(3.10, 3.40), new Translation2d(3.30, 4.60)),
            new Pair<Translation2d, Translation2d>(new Translation2d(13.40, 3.40), new Translation2d(13.50, 4.70)),
            new Pair<Translation2d, Translation2d>(new Translation2d(2.90, 3.50), new Translation2d(3.10, 4.50)),
            new Pair<Translation2d, Translation2d>(new Translation2d(13.50, 3.50), new Translation2d(13.60, 4.70)),
            new Pair<Translation2d, Translation2d>(new Translation2d(2.80, 3.60), new Translation2d(2.90, 4.40)),
            new Pair<Translation2d, Translation2d>(new Translation2d(13.60, 3.60), new Translation2d(13.70, 4.60)),
            new Pair<Translation2d, Translation2d>(new Translation2d(2.70, 3.70), new Translation2d(2.80, 4.20)),
            new Pair<Translation2d, Translation2d>(new Translation2d(0.00, 3.80), new Translation2d(0.10, 7.10)),
            new Pair<Translation2d, Translation2d>(new Translation2d(13.70, 3.80), new Translation2d(13.80, 4.40)),
            new Pair<Translation2d, Translation2d>(new Translation2d(16.40, 3.80), new Translation2d(16.60, 7.00)),
            new Pair<Translation2d, Translation2d>(new Translation2d(0.10, 3.90), new Translation2d(0.20, 6.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(13.80, 3.90), new Translation2d(13.90, 4.20)),
            new Pair<Translation2d, Translation2d>(new Translation2d(16.30, 3.90), new Translation2d(16.40, 6.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(0.20, 4.00), new Translation2d(0.40, 6.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(16.10, 4.00), new Translation2d(16.30, 6.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(16.00, 4.10), new Translation2d(16.10, 6.80)),
            new Pair<Translation2d, Translation2d>(new Translation2d(0.40, 4.20), new Translation2d(0.50, 6.80)),
            new Pair<Translation2d, Translation2d>(new Translation2d(15.80, 4.20), new Translation2d(16.00, 6.80)),
            new Pair<Translation2d, Translation2d>(new Translation2d(0.50, 4.30), new Translation2d(0.60, 6.80)),
            new Pair<Translation2d, Translation2d>(new Translation2d(0.60, 4.30), new Translation2d(0.70, 6.70)),
            new Pair<Translation2d, Translation2d>(new Translation2d(15.60, 4.30), new Translation2d(15.80, 6.70)),
            new Pair<Translation2d, Translation2d>(new Translation2d(0.70, 4.40), new Translation2d(0.80, 6.70)),
            new Pair<Translation2d, Translation2d>(new Translation2d(0.80, 4.40), new Translation2d(0.90, 6.60)),
            new Pair<Translation2d, Translation2d>(new Translation2d(15.40, 4.40), new Translation2d(15.60, 6.50)),
            new Pair<Translation2d, Translation2d>(new Translation2d(0.90, 4.60), new Translation2d(1.00, 6.60)),
            new Pair<Translation2d, Translation2d>(new Translation2d(1.00, 4.60), new Translation2d(1.10, 6.50)),
            new Pair<Translation2d, Translation2d>(new Translation2d(3.20, 4.60), new Translation2d(3.30, 4.70)),
            new Pair<Translation2d, Translation2d>(new Translation2d(15.30, 4.60), new Translation2d(15.40, 6.40)),
            new Pair<Translation2d, Translation2d>(new Translation2d(15.20, 4.70), new Translation2d(15.30, 5.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(1.10, 4.80), new Translation2d(1.20, 6.40)),
            new Pair<Translation2d, Translation2d>(new Translation2d(3.40, 4.80), new Translation2d(3.60, 4.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(1.20, 4.90), new Translation2d(1.30, 6.10)),
            new Pair<Translation2d, Translation2d>(new Translation2d(15.10, 4.90), new Translation2d(15.20, 5.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(1.30, 5.10), new Translation2d(1.40, 5.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(3.90, 5.20), new Translation2d(4.00, 5.30)),
            new Pair<Translation2d, Translation2d>(new Translation2d(4.10, 5.30), new Translation2d(4.20, 5.40)),
            new Pair<Translation2d, Translation2d>(new Translation2d(4.60, 5.50), new Translation2d(4.70, 5.60)),
            new Pair<Translation2d, Translation2d>(new Translation2d(5.10, 5.80), new Translation2d(5.20, 5.90)),
            new Pair<Translation2d, Translation2d>(new Translation2d(5.60, 6.10), new Translation2d(5.90, 6.20)),
            new Pair<Translation2d, Translation2d>(new Translation2d(15.50, 6.50), new Translation2d(15.60, 6.60))
        ), new Translation2d());
    }
}
