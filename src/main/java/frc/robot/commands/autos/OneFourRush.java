package frc.robot.commands.autos;

import static frc.robot.constants.AutoConstants.PathFollowConstants.DEFAULT_POSE_DELTA;
import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.AUTO_LINE;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.HOME;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.AutoShootState;

public class OneFourRush extends SequentialCommandGroup {
    public OneFourRush(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(
            new InstantCommand(drivetrain::disableVisionFusion),
            new InstantCommand(drivetrain::resetModules),
            new GoToArmElevatorState(armElevator, HOME),
            new PathFollowWithEvents(
                new PathFollowState(
                    drivetrain, 
                    getGlobalTrajectories().P_14, 
                    true, 
                    false
                ), 
                getGlobalTrajectories().P_14
            )
        );
    }
}
