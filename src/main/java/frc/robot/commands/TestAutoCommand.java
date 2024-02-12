package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;

public class TestAutoCommand extends SequentialCommandGroup {
    public TestAutoCommand(Drivetrain drivetrain) {
        super(
            new PathFollowState(drivetrain, getGlobalTrajectories().TEST_PATH, false, false).repeatedly()
        );
    }
}
