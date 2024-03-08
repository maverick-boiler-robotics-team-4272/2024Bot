package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.states.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.LidarStoppedFeedState;
import frc.robot.subsystems.shooter.states.LidarStoppedOutfeedState;


public class IntakeFeedCommand extends SequentialCommandGroup {
    public IntakeFeedCommand(IntakeSubsystem intake, Shooter shooter, DoubleSupplier power) {
        super(
            new ParallelRaceGroup(
                new IntakeState(intake, power),
                new LidarStoppedFeedState(shooter, power)
            ),
            new LidarStoppedOutfeedState(shooter, () -> 0.1)
        );
    }
}
