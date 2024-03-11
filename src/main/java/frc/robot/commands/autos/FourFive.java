package frc.robot.commands.autos;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;

public class FourFive extends SequentialCommandGroup {
    public FourFive(Drivetrain drivetrain) {
        super(
            new InstantCommand(() -> {
                drivetrain.disableVisionFusion();
                drivetrain.resetModules();
            }),
            new PathFollowWithEvents(
                new PathFollowState(
                    drivetrain, 
                    getGlobalTrajectories().P_45,
                    true,
                    false
                ), 
                getGlobalTrajectories().P_45
            )
        );
    }
}
