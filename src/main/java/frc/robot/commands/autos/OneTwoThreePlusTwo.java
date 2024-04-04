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
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.SUB_SHOT;

public class OneTwoThreePlusTwo extends SequentialCommandGroup {
    public OneTwoThreePlusTwo(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(
            new InstantCommand(drivetrain::disableVisionFusion),
            new InstantCommand(drivetrain::resetModules),
            new ParallelDeadlineGroup(
                new AutoShootState(shooter, 1.0, 1.0),
                new GoToArmElevatorState(armElevator, SUB_SHOT)
            ), //Arm goes home in path
            new PathFollowWithEvents(
                new PathFollowState(
                    drivetrain, 
                    getGlobalTrajectories().P_123PLUS_SUB,
                    true,
                    false
                ).withEndConditions(
                    true, 
                    false, 
                    DEFAULT_POSE_DELTA), 
                getGlobalTrajectories().P_123PLUS_SUB
            ),
            new ParallelRaceGroup(
                new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0),
                new AutoShootState(shooter, 1.0, 1.0)
            ),
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
