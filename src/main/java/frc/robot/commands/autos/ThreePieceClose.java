package frc.robot.commands.autos;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.PathFollowWithAimCommand;
import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.AutoShootState;
import frc.robot.subsystems.shooter.states.LidarStoppedFeedState;
import frc.robot.subsystems.shooter.states.OutfeedState;
import frc.robot.subsystems.shooter.states.RevState;

public class ThreePieceClose extends SequentialCommandGroup {
    public ThreePieceClose(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        PathFollowWithAimCommand aim = new PathFollowWithAimCommand(
            drivetrain, 
            armElevator, 
            getGlobalTrajectories().THREE_PIECE_CLOSE.trajectory
        );

        PathFollowWithEvents events = new PathFollowWithEvents(aim, getGlobalTrajectories().THREE_PIECE_CLOSE);

        events.addPauseTime(0.4, new SequentialCommandGroup(
            new InstantCommand(aim::pausePathFollowing),
            new ParallelRaceGroup(
                new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0),
                new RevState(shooter, 1.0).withTimeout(0.75).andThen(
                    new InstantCommand(drivetrain::enableVisionFusion),
                    new AutoShootState(shooter, 1.0, 1.0)
                )
            ),
            new InstantCommand(() -> {
                drivetrain.disableVisionFusion();
                aim.unpausePathFollowing();
                events.unpause();
            })
        ));
        
        events.addPauseTime(2.5, new SequentialCommandGroup(
            new InstantCommand(() -> {
                aim.pausePathFollowing();
                drivetrain.enableVisionFusion();
            }),
            new ParallelRaceGroup(
                new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0),
                new LidarStoppedFeedState(shooter, () -> 0.8).andThen(
                    new OutfeedState(shooter, () -> 0.2).withTimeout(0.2),
                    new WaitCommand(0.1),
                    new AutoShootState(shooter, 1.0, 1.0).withTimeout(5)
                )
            ),
            new InstantCommand(() -> {
                drivetrain.disableVisionFusion();
                aim.unpausePathFollowing();
                events.unpause();
            })
        ));

        events.addPauseTime(4.5, new SequentialCommandGroup(
            new InstantCommand(() -> {
                aim.pausePathFollowing();
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
                aim.unpausePathFollowing();
                events.unpause();
            })
        ));

        events.addPauseTime(5.2, new SequentialCommandGroup(
            new InstantCommand(() -> {
                aim.pausePathFollowing();
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
                aim.unpausePathFollowing();
                events.unpause();
            })
        ));

        events.addPauseTime(9.0, new SequentialCommandGroup(
            new InstantCommand(() -> {
                aim.pausePathFollowing();
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
                aim.unpausePathFollowing();
                events.unpause();
            })
        ));

        addCommands(
            new InstantCommand(drivetrain::disableVisionFusion),
            new InstantCommand(drivetrain::resetModules),
            events
        );
    }
}
