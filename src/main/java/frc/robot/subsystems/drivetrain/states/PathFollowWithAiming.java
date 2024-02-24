package frc.robot.subsystems.drivetrain.states;

import static frc.robot.constants.UniversalConstants.SPEAKER_SHOT_POSITION;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class PathFollowWithAiming extends PathFollowState {
    private Translation2d target;
    
    public PathFollowWithAiming(Drivetrain drivetrain, PathPlannerTrajectory path, Translation2d target) {
        super(drivetrain, path, true, false);

        this.target = target;
    }

    public PathFollowWithAiming(Drivetrain drivetrain, PathPlannerTrajectory path) {
        this(drivetrain, path, SPEAKER_SHOT_POSITION.toTranslation2d());
    }

    @Override
    public Rotation2d getDesiredTheta() {
        return target.minus(requiredSubsystem.getRobotPose().getTranslation()).getAngle();
    }
}
