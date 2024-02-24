package frc.robot.commands;

import static frc.robot.constants.RobotConstants.ArmElevatorSetpoints.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.armelevator.ArmElevatorSubsystem;
import frc.robot.subsystems.armelevator.states.GoToArmElevatorState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.LidarStoppedFeedState;

public class LoadCommand extends SequentialCommandGroup {
    public LoadCommand(Shooter shooter, ArmElevatorSubsystem armElevator) {
        super(
            new GoToArmElevatorState(armElevator, HOME),
            new LidarStoppedFeedState(shooter, () -> 0.5)
        );

        addRequirements(shooter, armElevator);
    }
}
