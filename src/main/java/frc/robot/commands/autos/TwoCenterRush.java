package frc.robot.commands.autos;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.AUTO_LINE;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.PathFollowWithAimCommand;
import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.AutoShootState;
import frc.robot.subsystems.shooter.states.LidarStoppedFeedState;
import frc.robot.subsystems.shooter.states.OutfeedState;
import frc.robot.subsystems.shooter.states.ShootState;

public class TwoCenterRush extends SequentialCommandGroup {
    public TwoCenterRush(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        PathFollowWithAimCommand aimingPath = new PathFollowWithAimCommand(
            drivetrain, armElevator, 
            getGlobalTrajectories().TWO_CENTER_RUSH.trajectory,
            shooter::lidarTripped
        );
        PathFollowWithEvents eventPath = new PathFollowWithEvents(
            aimingPath, 
            getGlobalTrajectories().TWO_CENTER_RUSH
        );

        eventPath.addPauseTime(1.75, new SequentialCommandGroup(
            new InstantCommand(() -> {
                aimingPath.pausePathFollowing();
                drivetrain.enableVisionFusion();
            }),
            new ParallelRaceGroup(
                new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0),
                new LidarStoppedFeedState(shooter, () -> 0.8).andThen(
                    new OutfeedState(shooter, () -> 0.2).withTimeout(0.2),
                    new WaitCommand(0.1),
                    new AutoShootState(shooter, 1.0, 1.0)
                )
            ),
            new InstantCommand(() -> {
                drivetrain.disableVisionFusion();
                aimingPath.unpausePathFollowing();
                eventPath.unpause();
            })
        ));

        eventPath.addPauseTime(7.25, new SequentialCommandGroup(
            new InstantCommand(() -> {
                aimingPath.pausePathFollowing();
                drivetrain.enableVisionFusion();
            }),
            new ParallelCommandGroup(
                new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0).withTimeout(0.35),
                new AutoShootState(shooter, 1.0, 1.0)
            ),
            new InstantCommand(() -> {
                drivetrain.disableVisionFusion();
                aimingPath.unpausePathFollowing();
                eventPath.unpause();
            })
        ));

        addCommands(
            new InstantCommand(drivetrain::disableVisionFusion),
            new InstantCommand(drivetrain::resetModules),
            new GoToArmElevatorState(armElevator, AUTO_LINE).withTimeout(0.25),
            new AutoShootState(shooter, 1.0, 1.0),
            eventPath
        );
    }
}
