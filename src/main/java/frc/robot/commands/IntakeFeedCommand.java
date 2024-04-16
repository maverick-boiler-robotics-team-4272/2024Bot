package frc.robot.commands;

// Commands / States
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.states.IntakeState;
import frc.robot.subsystems.shooter.states.LidarStoppedFeedState;
import frc.robot.subsystems.shooter.states.LidarStoppedOutfeedState;

// Subsystems
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;

import static frc.robot.utils.misc.BEAN.*;

public class IntakeFeedCommand extends SequentialCommandGroup {
    public IntakeFeedCommand(IntakeSubsystem intake, Shooter shooter, double power) {
        super(
            new ParallelRaceGroup(
                new IntakeState(intake, power),
                new LidarStoppedFeedState(shooter, power)
            ),
            new LidarStoppedOutfeedState(shooter, LIMA_BEAN.beans)
        );
    }
}
