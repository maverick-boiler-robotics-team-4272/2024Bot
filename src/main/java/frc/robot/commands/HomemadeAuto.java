package frc.robot.commands;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;

public class HomemadeAuto extends SequentialCommandGroup {
    public HomemadeAuto(Drivetrain drivetrain) {
        super(
            new PathFollowState(drivetrain, getGlobalTrajectories().HOMEMADE_PATH)
        );
    }
}
