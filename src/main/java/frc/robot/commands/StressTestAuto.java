package frc.robot.commands;

import static frc.robot.constants.AutoConstants.PathFollowConstants.DEFAULT_POSE_DELTA;
import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;

public class StressTestAuto extends SequentialCommandGroup {
    public StressTestAuto(Drivetrain drivetrain) {
      super(
        new PathFollowState(drivetrain, getGlobalTrajectories().STRESS_TEST_PATH.trajectory, false, true, DEFAULT_POSE_DELTA, true, false).repeatedly()
      );  
    }
}
