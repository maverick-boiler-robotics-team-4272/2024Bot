package frc.robot.utils;

import static frc.robot.constants.RobotConstants.DrivetrainConstants.*;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TrajectoryBuilder {
    public static PathPlannerPath buildPath(Pose2d start, Pose2d end) {
        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.bezierFromPoses(new Pose2d(start.getTranslation(), start.minus(end).getRotation()), end), 
            new PathConstraints(
                MAX_TRANSLATIONAL_SPEED, 
                3 * 9.81, //3g's but sidways
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
}
