package frc.robot.commands;

// Commands / States
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.states.PathFollowState;

// Subsystem
import frc.robot.subsystems.drivetrain.Drivetrain;

// Constants
import static frc.robot.constants.AutoConstants.PathFollowConstants.DEFAULT_POSE_DELTA;
import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;

public class StressTestAuto extends SequentialCommandGroup {
    public StressTestAuto(Drivetrain drivetrain) {
      super(
        new PathFollowState(drivetrain, getGlobalTrajectories().STRESS_TEST_PATH.trajectory, false, true, DEFAULT_POSE_DELTA, true, false).repeatedly()
      );  
    }
}
