package frc.robot.commands;

import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.TRAP;
import static frc.robot.constants.UniversalConstants.getGlobalPositions;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.states.GoToPositionState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.RevAndImbalancedShootState;

public class AutoTrapCommand extends SequentialCommandGroup {
    public AutoTrapCommand(Drivetrain drivetrain, ArmElevatorSubsystem armElevator, Shooter shooter, BooleanSupplier feed) {
        super(
            // new GoToPositionState(drivetrain, getGlobalPositions().TRAP_STAGE_POSE),
            new ParallelCommandGroup(
                new GoToArmElevatorState(armElevator, TRAP),
                new RevAndImbalancedShootState(shooter, 1.0, 0.8, 1.0, feed)
            )
        );
    }
}
