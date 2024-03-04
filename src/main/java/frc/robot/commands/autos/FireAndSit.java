package frc.robot.commands.autos;

import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.HOME;
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.SUB_SHOT;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.AutoShootState;

public class FireAndSit extends SequentialCommandGroup {
    public FireAndSit(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter) {
        super(
            new ParallelCommandGroup(
                new GoToArmElevatorState(armElevator, SUB_SHOT),
                new WaitCommand(0.75).andThen(
                    new AutoShootState(shooter, 0.9, 0.5)
                )
            ),
            new GoToArmElevatorState(armElevator, HOME),
            new InstantCommand(drivetrain::resetModules)
            // new DriveState(drivetrain, () -> 0.5, () -> 0.2, () -> 0.0, false)
        );
    }
}
