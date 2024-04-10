package frc.robot.commands.autos.state;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;

public class DisruptorAuto extends SequentialCommandGroup {
    public DisruptorAuto(Drivetrain drivetrain) {
        super(
            new InstantCommand(() -> {
                drivetrain.resetModules();
                drivetrain.disableVisionFusion();
            }),
            new PathFollowWithEvents(
                new PathFollowState(
                    drivetrain, 
                    getGlobalTrajectories().CHAOS,
                    true,
                    false
                ),
                getGlobalTrajectories().CHAOS
            )
        );
    }
}
