package frc.robot.subsystems.drivetrain.states;

import static frc.robot.constants.UniversalConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.paths.TrajectoryContainer.Path;

public class PathFollowWithAiming extends PathFollowState {
    private Translation2d target;
    
    public PathFollowWithAiming(Drivetrain drivetrain, Path path, Translation2d target) {
        super(drivetrain, path, true, false);

        this.target = target;
    }

    public PathFollowWithAiming(Drivetrain drivetrain, Path path) {
        this(drivetrain, path, getGlobalPositions().SPEAKER_SHOT_POSITION.toTranslation2d());
    }

    @Override
    public Rotation2d getDesiredTheta() {
        return target.minus(requiredSubsystem.getRobotPose().getTranslation()).getAngle();
    }
}
