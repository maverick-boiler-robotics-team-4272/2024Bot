package frc.robot.commands;

// Commands / States
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.armelevator.states.TargetPositionState;
import frc.robot.subsystems.drivetrain.states.PathFollowWithAiming;
import frc.robot.utils.misc.Pausable;
import frc.robot.utils.paths.Path;
// Subsystems
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;

// Utilities
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.BooleanSupplier;

// Constants
import static frc.robot.constants.UniversalConstants.*;

public class PathFollowWithAimCommand extends ParallelCommandGroup implements Pausable {
    private PathFollowWithAiming pathFollowCommand;

    public PathFollowWithAimCommand(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Path path, Translation3d target, BooleanSupplier lidar) {
        addCommands(
            (pathFollowCommand = new PathFollowWithAiming(drivetrain, path, target.toTranslation2d())),
            new TargetPositionState(armElevator, () -> drivetrain.getRobotPose().getTranslation(), target, lidar)
        );
    }

    public PathFollowWithAimCommand(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Path path, BooleanSupplier lidar) {
        this(drivetrain, armElevator, path, getGlobalPositions().SPEAKER_SHOT_POSITION, lidar);
    }

    public PathFollowWithAimCommand(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Path path) {
        this(drivetrain, armElevator, path, getGlobalPositions().SPEAKER_SHOT_POSITION, () -> true);
    }

    public void pause() {
        pathFollowCommand.pause();
    }

    public void unpause() {
        pathFollowCommand.unpause();
    }
}
