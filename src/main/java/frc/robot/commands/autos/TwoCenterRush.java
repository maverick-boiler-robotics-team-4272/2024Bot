package frc.robot.commands.autos;


// Commands / States
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.armelevator.states.*;
import frc.robot.subsystems.shooter.states.*;

// Subsystems
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

// Constants
import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.*;

public class TwoCenterRush extends SequentialCommandGroup {
    public TwoCenterRush(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        PathFollowWithAimCommand aimingPath = new PathFollowWithAimCommand(
            drivetrain, armElevator, 
            getGlobalTrajectories().TWO_CENTER_RUSH,
            shooter::endLidarTripped
        );
        PathFollowWithEvents eventPath = new PathFollowWithEvents(
            aimingPath, 
            getGlobalTrajectories().TWO_CENTER_RUSH
        );

        eventPath.addPauseTime(1.75, new SequentialCommandGroup(
            new InstantCommand(() -> {
                aimingPath.pause();
                drivetrain.enableVisionFusion();
            }),
            new ParallelRaceGroup(
                new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0),
                new LidarStoppedFeedState(shooter, 0.8).andThen(
                    new OutfeedState(shooter, 0.2).withTimeout(0.2),
                    new WaitCommand(0.1),
                    new AutoShootState(shooter, 1.0, 1.0)
                )
            ),
            new InstantCommand(() -> {
                drivetrain.disableVisionFusion();
                aimingPath.unpause();
                eventPath.unpause();
            })
        ));

        eventPath.addPauseTime(8.5, new SequentialCommandGroup(
            new InstantCommand(() -> {
                aimingPath.pause();
                drivetrain.enableVisionFusion();
            }),
            new ParallelRaceGroup(
                new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0),
                new LidarStoppedFeedState(shooter, 0.8).andThen(
                    new OutfeedState(shooter, 0.2).withTimeout(0.2),
                    new WaitCommand(0.1),
                    new AutoShootState(shooter, 1.0, 1.0)
                )
            ),
            new InstantCommand(() -> {
                drivetrain.disableVisionFusion();
                aimingPath.unpause();
                eventPath.unpause();
            })
        ));

        addCommands(
            new InstantCommand(drivetrain::disableVisionFusion),
            new InstantCommand(drivetrain::resetModules),
            new GoToArmElevatorState(armElevator, AUTO_LINE).withTimeout(0.25),
            new AutoShootState(shooter, 0.8, 1.0),
            eventPath
        );
    }
}
