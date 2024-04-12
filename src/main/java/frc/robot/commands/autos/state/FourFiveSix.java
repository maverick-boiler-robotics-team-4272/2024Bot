package frc.robot.commands.autos.state;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;
import frc.robot.subsystems.shooter.Shooter;

public class FourFiveSix extends SequentialCommandGroup {
    public FourFiveSix(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(
            new InstantCommand(() -> {
                drivetrain.resetModules();
                drivetrain.disableVisionFusion();
            }),
            new PathFollowWithEvents(
                new PathFollowState(
                    drivetrain, 
                    getGlobalTrajectories().P_456,
                    true,
                    false
                ), 
                getGlobalTrajectories().P_456
            )
        );
    }
}
