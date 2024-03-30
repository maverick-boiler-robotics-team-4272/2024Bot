package frc.robot.commands.autos;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.PathFollowState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.AutoShootState;

public class NoEight extends SequentialCommandGroup {
    public NoEight(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(
            new InstantCommand(() -> {
                drivetrain.resetModules();
                drivetrain.disableVisionFusion();
            }),
            new PathFollowWithEvents(
                new PathFollowState(drivetrain, getGlobalTrajectories().N_8, true, false),
                getGlobalTrajectories().N_8
            ),
            new ParallelRaceGroup(
                new AutoAimCommand(drivetrain, armElevator, () -> 0, () -> 0),
                new AutoShootState(shooter, 1.0, 1.0)
            ),
            new InstantCommand(() -> {
                drivetrain.enableVisionFusion();
            })
        );
    }
}
