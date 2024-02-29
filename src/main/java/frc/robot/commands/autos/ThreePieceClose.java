package frc.robot.commands.autos;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathFollowWithAimCommand;
import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class ThreePieceClose extends SequentialCommandGroup {
    public ThreePieceClose(Drivetrain drivetrain, ArmElevatorSubsystem armElevator) {
        PathFollowWithAimCommand aim = new PathFollowWithAimCommand(
            drivetrain, 
            armElevator, 
            getGlobalTrajectories().THREE_PIECE_CLOSE.trajectory
        );

        PathFollowWithEvents events = new PathFollowWithEvents(aim, getGlobalTrajectories().THREE_PIECE_CLOSE);

        addCommands(
            
        );
    }
}
