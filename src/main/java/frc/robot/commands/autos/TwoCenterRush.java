package frc.robot.commands.autos;

import static frc.robot.constants.AutoConstants.Paths.getGlobalTrajectories;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.AUTO_LINE;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PathFollowWithAimCommand;
import frc.robot.commands.PathFollowWithEvents;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.ShootState;

public class TwoCenterRush extends SequentialCommandGroup {
    public TwoCenterRush(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(
            new InstantCommand(drivetrain::disableVisionFusion),
            new ParallelRaceGroup(
                new GoToArmElevatorState(armElevator, AUTO_LINE).withTimeout(0.5),
                new ShootState(shooter, 1.0, 0.0)
            ),
            new ShootState(shooter, 1.0, 1.0).withTimeout(0.5),
            new PathFollowWithEvents(
                new PathFollowWithAimCommand(
                    drivetrain, armElevator, 
                    getGlobalTrajectories().TWO_CENTER_RUSH.trajectory,
                    shooter::lidarTripped
                ), 
                getGlobalTrajectories().TWO_CENTER_RUSH
            )
        );
    }
}
