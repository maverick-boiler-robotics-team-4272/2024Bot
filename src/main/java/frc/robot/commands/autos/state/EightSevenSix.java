package frc.robot.commands.autos.state;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.SUB_SHOT;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.AutoShootState;

public class EightSevenSix extends SequentialCommandGroup {
    public EightSevenSix(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(
            // new ParallelDeadlineGroup(
            //     new AutoShootState(shooter, 1.0, 1.0), 
            //     new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0)
            // ),
            new InstantCommand(() -> {
                drivetrain.resetModules();
                drivetrain.disableVisionFusion();
            }),
            new PathFollowWithEvents(
                new PathFollowState(
                    drivetrain, 
                    getGlobalTrajectories().P_876,
                    true,
                    false
                ), 
                getGlobalTrajectories().P_876
            )
        );
    }
}
