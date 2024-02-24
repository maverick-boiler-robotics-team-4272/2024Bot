package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.TargetPositionState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowWithAiming;

public class PathFollowWithAimCommand extends ParallelCommandGroup {
    public PathFollowWithAimCommand(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, PathPlannerTrajectory path, Translation3d target) {
        super(
            new PathFollowWithAiming(drivetrain, path, target.toTranslation2d()),
            new TargetPositionState(armElevator, () -> drivetrain.getRobotPose().getTranslation(), target)
        );
    }
}
