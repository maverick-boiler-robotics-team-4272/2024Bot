package frc.robot.commands;

// Constants / States
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.shooter.states.LidarStoppedFeedState;

// Subsystems
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.shooter.Shooter;

// Constants
import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.*;

public class LoadCommand extends SequentialCommandGroup {
    public LoadCommand(Shooter shooter, ArmElevatorSubsystem armElevator) {
        super(
            new GoToArmElevatorState(armElevator, HOME),
            new LidarStoppedFeedState(shooter, 0.5)
        );

        addRequirements(shooter, armElevator);
    }
}
