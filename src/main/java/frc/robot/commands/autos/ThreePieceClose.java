package frc.robot.commands.autos;

// Commands / States
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.shooter.states.*;

// Subsystems
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;
import frc.robot.subsystems.shooter.Shooter;

// Constants
import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;

public class ThreePieceClose extends SequentialCommandGroup {
    public ThreePieceClose(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(
            new InstantCommand(drivetrain::disableVisionFusion),
            new PathFollowWithEvents(
                new PathFollowState(
                    drivetrain, 
                    getGlobalTrajectories().THREE_PIECE_CLOSE,
                    true,
                    false
                ), 
                getGlobalTrajectories().THREE_PIECE_CLOSE
            )
        );
    }
}
