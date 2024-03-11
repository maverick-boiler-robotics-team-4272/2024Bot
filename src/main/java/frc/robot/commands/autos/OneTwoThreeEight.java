package frc.robot.commands.autos;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;

import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;

public class OneTwoThreeEight extends ThreePieceClose {
    public OneTwoThreeEight(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(drivetrain, armElevator, shooter);

        addCommands(
            new PathFollowWithEvents(
                new PathFollowState(
                    drivetrain, 
                    getGlobalTrajectories().P_1238,
                    false,
                    false
                ), 
                getGlobalTrajectories().P_1238
            )
        );
    }
}
