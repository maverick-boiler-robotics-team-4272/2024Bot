package frc.robot.commands.autos.worlds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;
import frc.robot.utils.paths.Path;

public class BaseAuto extends SequentialCommandGroup {
    protected final PathFollowWithEvents pathFollowCommand;

    public BaseAuto(Drivetrain drivetrain, boolean disable, Path path) {
        super(
            new InstantCommand(() -> {
                drivetrain.resetModules();
                if(disable)
                    drivetrain.disableVisionFusion();
            })
        );

        pathFollowCommand = new PathFollowWithEvents(
            new PathFollowState(drivetrain, path, true, false),
            path
        );
    }
}
