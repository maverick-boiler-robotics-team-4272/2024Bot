package frc.robot.commands.autos;

// Commands / States
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.shooter.states.*;

// Subsystems
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;
import frc.robot.subsystems.shooter.Shooter;

import static frc.robot.constants.AutoConstants.PathFollowConstants.DEFAULT_POSE_DELTA;
// Constants
import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.HOME;

public class OneTwoThree extends SequentialCommandGroup {
    public OneTwoThree(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(
            new InstantCommand(drivetrain::disableVisionFusion),
            new InstantCommand(drivetrain::resetModules),
            new GoToArmElevatorState(armElevator, HOME),
            new PathFollowWithEvents(
                new PathFollowState(
                    drivetrain, 
                    getGlobalTrajectories().THREE_PIECE_CLOSE,
                    true,
                    false
                ).withEndConditions(
                    true, 
                    false, 
                    DEFAULT_POSE_DELTA), 
                getGlobalTrajectories().THREE_PIECE_CLOSE
            ),
            new ParallelRaceGroup(
                new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0),
                new AutoShootState(shooter, 1.0, 1.0)
            )
        );
    }
}
