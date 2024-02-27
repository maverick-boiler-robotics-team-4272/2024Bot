package frc.robot.commands;

import static frc.robot.constants.UniversalConstants.SPEAKER_SHOT_POSITION;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.TargetPositionState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowWithAiming;

public class PathFollowWithAimCommand extends ParallelCommandGroup {
    public PathFollowWithAimCommand(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, PathPlannerTrajectory path, Translation3d target, BooleanSupplier lidar) {
        super(
            new PathFollowWithAiming(drivetrain, path, target.toTranslation2d()),
            new TargetPositionState(armElevator, () -> drivetrain.getRobotPose().getTranslation(), target, lidar)
        );
    }

    public PathFollowWithAimCommand(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, PathPlannerTrajectory path, BooleanSupplier lidar) {
        this(drivetrain, armElevator, path, SPEAKER_SHOT_POSITION, lidar);
    }

    public PathFollowWithAimCommand(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, PathPlannerTrajectory path) {
        this(drivetrain, armElevator, path, SPEAKER_SHOT_POSITION, () -> true);
    }
}
